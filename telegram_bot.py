"""
## Telegram Bot Setup

###  Create a Bot:
1. Open Telegram and search for @BotFather.
2. Type /newbot and follow the instructions.
3. Copy the Bot Token (e.g., 123456789:ABC-123xyz).

### Run the first part of this code to obtain your chat ID.

### Update Your Code:
Replace these lines in your Python script:
bot_token = "Place_Your_BOT_token_here"
chat_ids = ["Place_your_telegram_chatIDs_here"]

### Optional Testing:
You can test your bot by sending a simple message to your chat ID
before integrating it with the full script.
"""

#Getting Chat_ID from bot
import requests

TOKEN = "Place your bot token here"
url = f"https://api.telegram.org/bot{TOKEN}/getUpdates"

response = requests.get(url)
updates = response.json()
print(updates)



#Sending message via bot
import requests

TOKEN = "Place your bot token here"
CHAT_IDS = ["Place here your chat id's"] 
TEXT = "You can send message via your bot"

url = f"https://api.telegram.org/bot{TOKEN}/sendMessage"

for chat_id in CHAT_IDS:
    data = {"chat_id": chat_id, "text": TEXT}
    response = requests.post(url, data=data)
    print(f"Sent to {chat_id}: {response.json()}")




