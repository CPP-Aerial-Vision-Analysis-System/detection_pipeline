import subprocess
import os
import sys
import time
sys.path.append('..')
from pyodm import Node, exceptions

class ODMProcessor:
    def __init__(self, port=3000):
        self.port = port
        self.container_id = None
        self.node = None

    def start_container(self):
        """Start the Docker container for ODM processing"""

        # Check if container already exists and remove it
        check_process = subprocess.run(
            ["docker", "ps", "-a", "--filter", "name=nodeodm", "--format", "{{.ID}}"],
            capture_output=True,
            text=True
        )
        
        # Stop and remove any existing container
        if check_process.stdout.strip():
            print("Removing existing nodeodm container...")
            subprocess.run(
                ["docker", "rm", "-f", "nodeodm"], 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )

        # Start container
        process = subprocess.Popen(
            ["docker", "run", "--runtime", "nvidia", "-d", "-p", f"{self.port}:3000", "opendronemap/nodeodm"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        container_id, error = process.communicate()
        
        if error:
            raise RuntimeError(f"Error starting container: {error.decode()}")
            
        self.container_id = container_id.decode().strip()
        print(f"Docker container started with ID: {self.container_id}")
        
        # Wait for container initialization
        time.sleep(10)
        self.node = Node("localhost", self.port)

    def process_images(self, image_folder, source_folder='./results', options=None):
        """Process images using ODM"""
        if not self.node:
            raise RuntimeError("Container not started. Call start_container() first")

        options = {
            'dsm': True,
            'orthophoto-resolution': 2,
            'pc-quality': 'low',
            'fast-orthophoto': True,
            'skip-3dmodel': True,
            'skip-report' : True
        }
        
        # if options:
        #     default_options.update(options)

        image_paths = [os.path.join(image_folder, f) 
                      for f in os.listdir(image_folder) 
                      if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        
        try:
            if not image_paths:
                raise FileNotFoundError(f"No image files found in {image_folder}")

            print(f"Uploading {len(image_paths)} images...")

            task = self.node.create_task(image_paths, options)
            print(task.info())
            try:
                task.wait_for_completion()
                print("Task completed,, downloading results...")
                task.download_assets(source_folder)
            except exceptions.TaskFailedError as e:
                print("\n".join(task.output()))
        except exceptions.NodeConnectionError as e:
            print("Cannot Connect: %s" % e)
        except exceptions.NodeResponseError as e:
            print("Error: %s" % e)
        except FileNotFoundError as e:
            print(e)

    def stop_container(self):
        """Stop the Docker container"""
        if self.container_id:
            print(f"Stopping docker container {self.container_id}...")
            subprocess.run(['docker', 'stop', self.container_id])
            self.container_id = None
            self.node = None

    def __enter__(self):
        """Context manager entry"""
        self.start_container()
        return self

    def __exit__(self):
        """Context manager exit"""
        self.stop_container()