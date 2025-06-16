#!/usr/bin/env python3

import rospy
import sys
import cv2
import glob
import shutil
import numpy as np
import os
import rospkg
import time
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime
from mavros_msgs.msg import StatusText  # For MAVLink status messages


class ImageStitcher:
    def __init__(self, nfeatures=5000, min_matches=5, distance_threshold=0.6):
        self.orb = cv2.ORB_create(nfeatures=nfeatures, scaleFactor=1.2, nlevels=8)
        self.min_matches = min_matches
        self.distance_threshold = distance_threshold
        self.result = None

        self.node_name = "mapping"
        rospy.init_node(self.node_name, anonymous=True)
        self.message_sender = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.statustext_callback)

        rp = rospkg.RosPack()

        self.usb_mount_path = r'/media/astra/USB DISK'
        self.image_folder = os.path.join(rp.get_path("video_cam"), "mapping")
        #self.result_folder = os.path.join(rp.get_path("video_cam"), "result")
        self.result_folder = os.path.join(self.usb_mount_path, "results")
        Path(self.image_folder).mkdir(parents=True, exist_ok=True)
        Path(self.result_folder).mkdir(parents=True, exist_ok=True)

    def statustext_callback(self, msg):
        if "map" in msg.text.lower():
            rospy.logwarn("Received 'map' command. Starting stitching...")
            self.send_ack("Map command received. Starting stitching...")

            try:
                self.stitch_images(self.image_folder)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                result_file = Path(self.result_folder) / f"stitched_output_{timestamp}.jpg"
                self.save_result(str(result_file))
                self.send_ack(f"Stitching complete. Saved: {result_file.name}")
                time.sleep(3)
                self.unmount_usb()
            except Exception as e:
                rospy.logerr(f"Stitching failed: {e}")
                self.send_ack(f"Stitching failed: {e}")

    def send_ack(self, text):
        status_msg = StatusText()
        status_msg.severity = 6
        status_msg.text = text
        self.message_sender.publish(status_msg)

    def load_images(self, folder_path, max_size=2000):
        path = sorted(Path(folder_path).glob("*.jpg"))
        img_list = []

        print("Loading and resizing images...")
        for img_path in path:
            img = cv2.imread(str(img_path))
            if img is not None:
                height, width = img.shape[:2]
                if height > max_size or width > max_size:
                    scale = max_size / max(height, width)
                    new_size = (int(width * scale), int(height * scale))
                    img = cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)
                img_list.append(img)
                print(f"Loaded and processed: {img_path.name}")

        if len(img_list) < 2:
            raise ValueError("At least 2 images required for stitching")
        return img_list

    def stitch_images(self, folder_path):
        img_list = self.load_images(folder_path)
        print(f"Processing {len(img_list)} images...")

        while len(img_list) > 1:
            img1 = img_list.pop(0)
            img2 = img_list.pop(0)

            kp1, des1 = self.orb.detectAndCompute(img1, None)
            kp2, des2 = self.orb.detectAndCompute(img2, None)

            if des1 is None or des2 is None:
                print("Warning: No features detected in one or both images")
                continue

            # ✅ OpenCV 4.2-compatible brute-force matcher
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
            matches = bf.knnMatch(des1, des2, k=2)

            good = [m for m, n in matches if m.distance < self.distance_threshold * n.distance]

            if len(good) > self.min_matches:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

                M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                result = self.warp_images(img2, img1, M)
                img_list.insert(0, result)

        if len(img_list) == 1:
            self.result = cv2.cvtColor(img_list[0], cv2.COLOR_BGR2RGB)

    def warp_images(self, img1, img2, H):
        rows1, cols1 = img1.shape[:2]
        rows2, cols2 = img2.shape[:2]

        points1 = np.float32([[0, 0], [0, rows1], [cols1, rows1], [cols1, 0]]).reshape(-1, 1, 2)
        points2 = np.float32([[0, 0], [0, rows2], [cols2, rows2], [cols2, 0]]).reshape(-1, 1, 2)
        transformed_points2 = cv2.perspectiveTransform(points2, H)
        all_points = np.concatenate((points1, transformed_points2), axis=0)

        [x_min, y_min] = np.int32(all_points.min(axis=0).ravel() - 0.5)
        [x_max, y_max] = np.int32(all_points.max(axis=0).ravel() + 0.5)

        width = int(x_max - x_min)
        height = int(y_max - y_min)
        max_output_size = 10000 * 10000 # modify for higher limit, jetson low

        if width > 20000 or height > 20000 or (width * height > max_output_size): # max before runtime error
            print(f"⚠️ Skipping warp: estimated output size too large ({width}x{height})")
            return img1

        translation = [-x_min, -y_min]
        H_translation = np.array([[1, 0, translation[0]], [0, 1, translation[1]], [0, 0, 1]])

        result_img = cv2.warpPerspective(img2, H_translation @ H, (x_max - x_min, y_max - y_min))
        result_img[translation[1]:rows1 + translation[1], translation[0]:cols1 + translation[0]] = img1

        gray = cv2.cvtColor(result_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        x, y, w, h = cv2.boundingRect(thresh)
        return result_img[y:y + h, x:x + w]

    def save_result(self, output_path):
        if self.result is not None:
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(output_path, cv2.cvtColor(self.result, cv2.COLOR_RGB2BGR))
            rospy.loginfo(f"Result saved to {output_path}")
        else:
            raise ValueError("No result to save. Run stitching first.")

    def show_result(self):
        if self.result is not None:
            plt.figure(figsize=(15, 10))
            plt.imshow(self.result)
            plt.axis('off')
            plt.show()
        else:
            print("No result available. Run stitch_images first.")

    def unmount_usb(self):
        """Unmount the USB drive after stitching is complete."""
        try:
            #mount_point = "name of USB drive"  # Replace with actual mount point
            password = "UAV_Lab"
            os.system(f'echo {password} | sudo -S umount "{self.usb_mount_path}"')
            #os.system(f"sudo umount {mount_point}")
            message = "USB drive unmounted successfully."
            rospy.loginfo(message)
            self.send_ack(message)
        except Exception as e:
            message = f"Failed to unmount USB drive: {e}"
            rospy.logerr(message)
            self.send_ack(message)

def file_transfer(destination_folder, result_file):
    try:
        Path(destination_folder).mkdir(parents=True, exist_ok=True)
        destination_path = Path(destination_folder) / Path(result_file).name
        shutil.copy(result_file, destination_path)
        print(f"Result file transferred to: {destination_path}")
    except Exception as e:
        print(f"Error during file transfer: {e}")


if __name__ == "__main__":
    stitcher = ImageStitcher(
        nfeatures=2000,
        min_matches=5,
        distance_threshold=0.7
    )

    rospy.loginfo("Image stitcher node is ready. Send 'map' command via Mission Planner.")
    rospy.spin()