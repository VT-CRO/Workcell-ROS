import requests
import argparse
from mc2425.variables import DISCORD_NOTIFY_PATH, FRONTEND_URL, BOT_UUID

def send_notification(message):
    """
    Sends a notification to the backend server to @mention a Discord user and type a number.

    :message: The message to send to the channel
    """
    endpoint = f"{FRONTEND_URL}/api/{BOT_UUID}/{DISCORD_NOTIFY_PATH}"
    print(endpoint)

    payload = {"message":message}

    try:
        response = requests.post(endpoint, json=payload)

        # Check if the request was successful
        if response.status_code == 200:
            print("Notification sent successfully.")
        else:
            print(f"Failed to send notification. Status code: {response.status_code}")
            print("Response:", response.json())

    except requests.RequestException as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a notification to a Discord user.")
    parser.add_argument("-m", "--message", required=True, nargs="+", help="Message to send")
    args = parser.parse_args()
    message = " ".join(args.message)
    send_notification(message)
