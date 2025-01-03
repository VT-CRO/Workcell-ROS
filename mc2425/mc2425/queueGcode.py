import requests
import os
from mc2425.variables import FRONTEND_URL, BOT_UUID, QUEUE_SAVE

def download_gcode():
    # Construct the URL for the bot-specific route
    url = f"{FRONTEND_URL}/api/{BOT_UUID}/requestgcode"
    
    if not os.path.exists(QUEUE_SAVE):
        os.makedirs(QUEUE_SAVE)
        
    try:
        print(f"Sending request to: {url}")
        response = requests.get(url, stream=True)

        # Check if the request was successful
        if response.status_code == 200:
            # Extract filename from Content-Disposition header
            content_disposition = response.headers.get("Content-Disposition", "")
            filename = "downloaded_file.gcode"  # Default filename
            if "filename=" in content_disposition:
                filename = content_disposition.split("filename=")[1].strip().strip('"')

            savepath = os.path.join(QUEUE_SAVE, filename)
            # Write the file in chunks to avoid memory issues with large files
            with open(savepath, "wb") as file:
                for chunk in response.iter_content(chunk_size=8192):
                    file.write(chunk)
            print(f"File downloaded successfully and saved as '{filename}'")
            return filename
        elif response.status_code == 404:
            print("Error: The queue is empty or no file to download.")
            return -1
        elif response.status_code == 500:
            print("Error: File not found on the server.")
            return -1
        else:
            print(f"Error: Received unexpected status code {response.status_code}")
            print(response.json())  # Print server error details if available
            return -1

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        return -1

if __name__ == "__main__":
    download_gcode()