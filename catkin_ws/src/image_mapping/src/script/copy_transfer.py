import os
import time
import shutil

class ResultsTransfer:
    def __init__(self, source_folder='./results', destination_folder='D:/test_results', retry_delay=5):
        self.source_folder = source_folder
        self.destination_folder = destination_folder
        self.retry_delay = retry_delay

    def setup_destination(self):
        """Create destination folder if it doesn't exist"""
        os.makedirs(self.destination_folder, exist_ok=True)

    def transfer_files(self):
        """Transfer files from source to destination with retry logic"""
        self.setup_destination()
        
        try:
            shutil.copytree(self.source_folder, 
                           self.destination_folder, 
                           dirs_exist_ok=True)
            print(f"Results successfully copied to {self.destination_folder}")
            return True
        except PermissionError as e:
            print(f"Permission error: {e}")
            print(f"Retrying in {self.retry_delay} seconds...")
            time.sleep(self.retry_delay)
            try:
                shutil.copytree(self.source_folder, 
                               self.destination_folder, 
                               dirs_exist_ok=True)
                print(f"Results successfully copied to {self.destination_folder}")
                return True
            except Exception as e:
                print(f"Failed to transfer files after retry: {e}")
                return False

if __name__ == '__main__':
    transfer = ResultsTransfer()
    transfer.transfer_files()