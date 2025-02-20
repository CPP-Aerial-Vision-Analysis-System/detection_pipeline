
import numpy as np
import os
from PIL import Image
from datetime import datetime
import math

class WaypointsUtils:
    def __init__(self, lon_x, lat_y, alt_z, obj_detected):
        self.lon_x = lon_x
        self.lat_y = lat_y
        self.alt_z = alt_z
        self.obj_detected = obj_detected
        #self.calc_x
        #Orientation -> published topics: /mavros/local_position/pose

        self.calculate_gps_calc()

    def get_image_dimensions(self, img_path):
        """
        Given an image file path, return a tuple (width, height) if the image exists and can be opened.
        Returns None if the file does not exist or is not a valid image.
        """
        if os.path.exists(img_path):
            try:
                with Image.open(img_path) as img:
                    return img.size  # (width, height)
            except Exception as e:
                print(f"Error reading image '{img_path}': {e}")
                return None
        else:
            print(f"Image file '{img_path}' does not exist.")
            return None

    # def calculate_gps_calc(self, gps_lat, gps_lon, target_x, target_y, img_width, img_height, fov_width, fov_height, altitude):
    #     """
    #     Calculate the target GPS position based on:
    #     - Current GPS location (gps_lat, gps_lon)
    #     - Target's pixel location (target_x, target_y)
    #     - Image dimensions (img_width, img_height)
    #     - Camera's FOV (fov_width, fov_height)
    #     - Drone altitude (altitude)
        
    #     Returns:
    #         new_gps_lat, new_gps_lon
    #     """
    #     # Convert FOV from degrees to radians
    #     fov_width_rad = math.radians(fov_width)
    #     fov_height_rad = math.radians(fov_height)

    #     # Calculate ground footprint at given altitude
    #     ground_width = 2 * altitude * math.tan(fov_width_rad / 2)
    #     ground_height = 2 * altitude * math.tan(fov_height_rad / 2)

    #     # Calculate meters per pixel
    #     meters_per_pixel_x = ground_width / img_width
    #     meters_per_pixel_y = ground_height / img_height

    #     # Determine pixel offset from image center
    #     image_center_x = img_width / 2.0
    #     image_center_y = img_height / 2.0

    #     dx_pixels = target_x - image_center_x
    #     dy_pixels = image_center_y - target_y

    #     # Convert pixel offset to meters
    #     dx_meters = dx_pixels * meters_per_pixel_x
    #     dy_meters = dy_pixels * meters_per_pixel_y

    #     # Convert meters to GPS coordinates
    #     meters_per_degree_lat = 111320  # Approximate
    #     meters_per_degree_lon = 111320 * math.cos(math.radians(gps_lat))

    #     new_gps_lat = gps_lat + (dy_meters / meters_per_degree_lat)
    #     new_gps_lon = gps_lon + (dx_meters / meters_per_degree_lon)

    #     return new_gps_lat, new_gps_lon


    def gps_calc(self, gps_lat, gps_lon, target_x, target_y, img_width, img_height, fov_width, fov_height, altitude, yaw_degrees):
        """
        Calculate the GPS coordinates of a target point in an image taken from a drone, accounting for:
        - Drone's GPS position (gps_lat, gps_lon)
        - Target's pixel position in the image (target_x, target_y)
        - Camera specifications (img_width, img_height, fov_width, fov_height)
        - Drone's altitude
        - Drone's yaw orientation (yaw_degrees, 0° = North)

        Returns:
            new_gps_lat, new_gps_lon
        """
        # Convert FOV from degrees to radians
        fov_width_rad = math.radians(fov_width)
        fov_height_rad = math.radians(fov_height)

        # Compute ground coverage based on altitude
        ground_width = 2 * altitude * math.tan(fov_width_rad / 2)
        ground_height = 2 * altitude * math.tan(fov_height_rad / 2)

        # Calculate meters per pixel
        meters_per_pixel_x = ground_width / img_width
        meters_per_pixel_y = ground_height / img_height

        # Compute pixel displacement from the image center
        image_center_x = img_width / 2.0
        image_center_y = img_height / 2.0

        dx_pixels = target_x - image_center_x
        dy_pixels = image_center_y - target_y  # Invert because image Y axis is top-down

        # Convert pixel displacement to real-world displacement (in meters)
        dx_meters = dx_pixels * meters_per_pixel_x
        dy_meters = dy_pixels * meters_per_pixel_y

        # Convert yaw from degrees to radians for rotation
        yaw_radians = math.radians(yaw_degrees)

        # Rotate displacement based on drone yaw (2D rotation matrix)
        rotated_dx = dx_meters * math.cos(yaw_radians) - dy_meters * math.sin(yaw_radians)
        rotated_dy = dx_meters * math.sin(yaw_radians) + dy_meters * math.cos(yaw_radians)

        # Convert meters to GPS degrees
        meters_per_degree_lat = 111320  # Approximate meters per degree latitude
        meters_per_degree_lon = 111320 * math.cos(math.radians(gps_lat))  # Adjust for longitude scaling

        # Apply rotated displacement to GPS coordinates
        new_gps_lat = gps_lat + (rotated_dy / meters_per_degree_lat)
        new_gps_lon = gps_lon + (rotated_dx / meters_per_degree_lon)

        return new_gps_lat, new_gps_lon

    
    # def main():
    #     """
    #     Console menu for testing the functions.
    #     Each option will check for an empty row id and create a new row if needed.
    #     """
    #     while True:
    #         print("\nOptions:")
    #         print("1. Save GPS lat, lon, alt")
    #         print("2. Save image directory (with automatic dimension update)")
    #         print("3. Save image dimensions")
    #         print("4. Save image target x, y")
    #         print("5. Save camera FoV dimensions")
    #         print("6. Calculate and save target GPS position")
    #         print("7. Update image dimensions from existing image directory")
    #         print("8. Exit")
            
    #         choice = input("Select an option: ").strip()
            
    #         try:
    #             if choice == "1":
    #                 row_id = self.get_row_id_input()
    #                 gps_lat = float(input("Enter gps_lat_x: "))
    #                 gps_lon = float(input("Enter gps_lon_y: "))
    #                 gps_alt = float(input("Enter gps_alt_z: "))
    #                 save_gps_latlon_alt(row_id, gps_lat, gps_lon, gps_alt)
    #             elif choice == "2":
    #                 row_id = get_row_id_input()
    #                 img_directory = input("Enter image directory: ").strip()
    #                 save_img_directory(row_id, img_directory)
    #             elif choice == "3":
    #                 row_id = get_row_id_input()
    #                 img_width = float(input("Enter image width: "))
    #                 img_height = float(input("Enter image height: "))
    #                 save_img_dimensions(row_id, img_width, img_height)
    #             elif choice == "4":
    #                 row_id = get_row_id_input()
    #                 img_target_x = float(input("Enter image target x: "))
    #                 img_target_y = float(input("Enter image target y: "))
    #                 save_img_target(row_id, img_target_x, img_target_y)
    #             elif choice == "5":
    #                 row_id = get_row_id_input()
    #                 cam_fov_width = float(input("Enter camera FoV width: "))
    #                 cam_fov_height = float(input("Enter camera FoV height: "))
    #                 save_cam_fov(row_id, cam_fov_width, cam_fov_height)
    #             elif choice == "6":
    #                 row_id = get_row_id_input()
    #                 calculate_and_save_target_position(row_id)
    #             elif choice == "7":
    #                 row_id = get_row_id_input("Enter row id to update image dimensions: ")
    #                 if row_id is not None:
    #                     update_img_dimensions_for_row(row_id)
    #                 else:
    #                     print("Row id must be provided for updating image dimensions.")
    #             elif choice == "8":
    #                 print("Exiting...")
    #                 break
    #             else:
    #                 print("Invalid option. Please try again.")
    #         except Exception as e:
    #             print(f"An error occurred: {e}")

# if __name__ == '__main__':
#     main()
