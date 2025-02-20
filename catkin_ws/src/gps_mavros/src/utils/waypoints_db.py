# import mysql.connector

class WaypointsDb:


    # def connect_db():
    #     """
    #     Connect to the MySQL database.
    #     Update the connection parameters with your own credentials.
    #     """
    #     conn = mysql.connector.connect(
    #         host='localhost',       # Replace with your host
    #         user='root',            # Replace with your username
    #         password='Pa$$w0rd',    # Replace with your password
    #         database='astra'
    #     )
    #     return conn

    # def update_or_insert(row_id, table, data):
    #     """
    #     If row_id is provided, update the given columns in the table.
    #     Otherwise, insert a new row with the provided data.
        
    #     Automatically adds/updates the 'time_stamp' field to the current datetime.
    #     Opens a new connection for this query and closes it after execution.
        
    #     Returns the row id (existing or new).
    #     """
    #     # Always update time_stamp to current time.
    #     data["time_stamp"] = datetime.now()
        
    #     conn = connect_db()
    #     cursor = conn.cursor()
    #     try:
    #         if row_id is not None:
    #             # Build update query dynamically based on the keys in data.
    #             set_clause = ", ".join([f"{col} = %s" for col in data.keys()])
    #             query = f"UPDATE {table} SET {set_clause} WHERE id = %s"
    #             values = list(data.values()) + [row_id]
    #             cursor.execute(query, values)
    #             conn.commit()
    #             new_id = row_id
    #         else:
    #             # Build insert query dynamically.
    #             columns = ", ".join(data.keys())
    #             placeholders = ", ".join(["%s"] * len(data))
    #             query = f"INSERT INTO {table} ({columns}) VALUES ({placeholders})"
    #             values = list(data.values())
    #             cursor.execute(query, values)
    #             conn.commit()
    #             new_id = cursor.lastrowid
    #     finally:
    #         cursor.close()
    #         conn.close()
    #     return new_id

    def save_gps_latlon_alt(self, row_id, gps_lat, gps_lon, gps_alt):
        """
        Save gps_lat_x, gps_lon_y, and gps_alt_z values for the given row.
        If row_id is None/empty, create a new row.
        """
        data = {'gps_lat_x': gps_lat, 'gps_lon_y': gps_lon, 'gps_alt_z': gps_alt}
        new_id = self.update_or_insert(row_id, "waypoints", data)
        print(f"Row id {new_id} now contains gps_lat_x, gps_lon_y, gps_alt_z values with updated time_stamp.")
        return new_id

    def save_img_directory(self, row_id, img_directory):
        """
        Save the image directory string for the given row.
        Also attempts to retrieve the image's dimensions (width/height) if the file exists,
        and then updates those columns in the table.
        If row_id is None/empty, a new row is created.
        """
        # First, update or insert the image directory.
        new_id = self.update_or_insert(row_id, "waypoints", {'img_directory': img_directory})
        print(f"Row id {new_id} now contains the image directory '{img_directory}' with updated time_stamp.")

        # Attempt to get image dimensions from the file system.
        dims = self.get_image_dimensions(img_directory)
        if dims is not None:
            img_width, img_height = dims
            self.update_or_insert(new_id, "waypoints", {'img_width': img_width, 'img_height': img_height})
            print(f"Row id {new_id} updated with image dimensions: width={img_width}, height={img_height}.")
        else:
            print("Image dimensions not updated because the file was not found or could not be read.")
        return new_id

    def save_img_dimensions(self, row_id, img_width, img_height):
        """
        Save image width and height for the given row.
        If row_id is None/empty, create a new row.
        """
        data = {'img_width': img_width, 'img_height': img_height}
        new_id = self.update_or_insert(row_id, "waypoints", data)
        print(f"Row id {new_id} now contains image dimensions with updated time_stamp.")
        return new_id

    def save_img_target(self, row_id, img_target_x, img_target_y):
        """
        Save the image target x and y coordinates for the given row.
        If row_id is None/empty, create a new row.
        """
        data = {'img_target_x': img_target_x, 'img_target_y': img_target_y}
        new_id = self.update_or_insert(row_id, "waypoints", data)
        print(f"Row id {new_id} now contains image target coordinates with updated time_stamp.")
        return new_id

    def save_cam_fov(self, row_id, cam_fov_width, cam_fov_height):
        """
        Save the camera field-of-view dimensions for the given row.
        If row_id is None/empty, create a new row.
        """
        data = {'cam_fov_width': cam_fov_width, 'cam_fov_height': cam_fov_height}
        new_id = self.update_or_insert(row_id, "waypoints", data)
        print(f"Row id {new_id} now contains camera FoV dimensions with updated time_stamp.")
        return new_id
    
    def save_gps_calc(self, row_id, gps_calc_x, gps_calc_y):
        """
        Save the calculated GPS values into the given row.
        These are saved in the columns gps_calc_x and gps_calc_y.
        If row_id is None/empty, create a new row.
        """
        data = {'gps_calc_x': gps_calc_x, 'gps_calc_y': gps_calc_y}
        new_id = self.update_or_insert(row_id, "waypoints", data)
        print(f"Row id {new_id} now contains calculated GPS values (gps_calc_x, gps_calc_y) with updated time_stamp.")
        return new_id
    
    def calculate_and_save_target_position(self, row_id):
        """
        Retrieve all necessary data from a specific row, calculate the target's GPS position,
        and save the calculated values (new_gps_lat_x, new_gps_lon_y) into that row (in gps_calc_x and gps_calc_y).
        
        Required fields in the row:
            - gps_lat_x, gps_lon_y (current GPS coordinates)
            - img_target_x, img_target_y (target location in image pixels)
            - img_width, img_height (dimensions of the image)
            - cam_fov_width, cam_fov_height (camera FoV dimensions in GPS units)
        
        If any required field is missing, the calculation is aborted.
        If row_id is None, the user is prompted to enter the necessary data to create a new row.
        """
        if row_id is None:
            print("No row id provided. Creating a new row by prompting for necessary data:")
            gps_lat = float(input("Enter gps_lat_x: "))
            gps_lon = float(input("Enter gps_lon_y: "))
            gps_alt = float(input("Enter gps_alt_z: "))
            img_target_x = float(input("Enter image target x: "))
            img_target_y = float(input("Enter image target y: "))
            img_width = float(input("Enter image width: "))
            img_height = float(input("Enter image height: "))
            cam_fov_width = float(input("Enter camera FoV width: "))
            cam_fov_height = float(input("Enter camera FoV height: "))
            data = {
                "gps_lat_x": gps_lat,
                "gps_lon_y": gps_lon,
                "gps_alt_z": gps_alt,
                "img_target_x": img_target_x,
                "img_target_y": img_target_y,
                "img_width": img_width,
                "img_height": img_height,
                "cam_fov_width": cam_fov_width,
                "cam_fov_height": cam_fov_height
            }
            row_id = self.update_or_insert(None, "waypoints", data)
            print(f"New row created with id {row_id}.")

        # Retrieve the row data for calculation.
        conn = self.connect_db()
        cursor = conn.cursor(dictionary=True)
        try:
            query = """
            SELECT gps_lat_x, gps_lon_y, img_target_x, img_target_y, img_width, img_height, 
                cam_fov_width AS fov_width, cam_fov_height AS fov_height
            FROM waypoints WHERE id = %s
            """
            cursor.execute(query, (row_id,))
            row = cursor.fetchone()
        finally:
            cursor.close()
            conn.close()

        if row is None:
            print(f"Row with id {row_id} not found.")
            return

        # Check for missing required fields.
        required_fields = ['gps_lat_x', 'gps_lon_y', 'img_target_x', 'img_target_y', 'img_width', 'img_height', 'fov_width', 'fov_height']
        missing = [field for field in required_fields if row.get(field) is None]
        if missing:
            print(f"Row with id {row_id} is missing the following required fields: {', '.join(missing)}. Cannot calculate target GPS position.")
            return

        gps_lat = row['gps_lat_x']
        gps_lon = row['gps_lon_y']
        target_x = row['img_target_x']
        target_y = row['img_target_y']
        img_width = row['img_width']
        img_height = row['img_height']
        fov_width = row['fov_width']
        fov_height = row['fov_height']

        new_gps_lat, new_gps_lon = self.calculate_gps_calc(gps_lat, gps_lon, target_x, target_y, img_width, img_height, fov_width, fov_height)
        self.save_gps_calc(row_id, new_gps_lat, new_gps_lon)
        print(f"Calculated target GPS position saved in row {row_id}: ({new_gps_lat}, {new_gps_lon})")

    def update_img_dimensions_for_row(self, row_id):
        """
        Retrieve the img_directory from the given row and, if it exists,
        automatically calculate the image's dimensions and update the row.
        """
        conn = self.connect_db()
        cursor = conn.cursor(dictionary=True)
        try:
            query = "SELECT img_directory FROM waypoints WHERE id = %s"
            cursor.execute(query, (row_id,))
            row = cursor.fetchone()
        finally:
            cursor.close()
            conn.close()

        if row is None:
            print(f"No row with id {row_id} exists.")
            return
        img_directory = row.get("img_directory")
        if not img_directory:
            print(f"Row {row_id} does not have an image directory set.")
            return
        dims = self.get_image_dimensions(img_directory)
        if dims is None:
            print("Could not retrieve image dimensions for the given directory.")
            return
        img_width, img_height = dims
        self.update_or_insert(row_id, "waypoints", {"img_width": img_width, "img_height": img_height})
        print(f"Row id {row_id} updated with image dimensions: width = {img_width}, height = {img_height}.")

    def get_row_id_input(self, prompt="Enter row id (leave empty for new row): "):
        """
        Utility function to get a row id input from the user.
        Returns an integer row id or None if the input is empty.
        """
        user_input = input(prompt).strip()
        if user_input == "":
            return None
        return int(user_input)
