from flask import Flask, render_template, request, jsonify
from logic.main import solve_maze_from_web
import requests
import os
from dotenv import load_dotenv
load_dotenv()

app = Flask(__name__)

FIREBASE_URL = os.getenv("FIREBASE_URL")
FIREBASE_SECRET = os.getenv("FIREBASE_SECRET")


def send_to_firebase(directions, start, end):
    payload = {
        "directions": directions,
        "start": start,
        "end": end
    }

    try:
        response = requests.put(f"{FIREBASE_URL}/solutions/latest.json?auth={FIREBASE_SECRET}", json=payload)
        print("✅ Sent to Firebase:", response.status_code)
    except Exception as e:
        print("❌ Error sending to Firebase:", e)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/solve", methods=["POST"])
def solve():
    data = request.get_json()
    result = solve_maze_from_web(
        maze_grid=data["maze_grid"],
        start_pos=data["start"],
        end_pos=data["end"]
    )

    # ✅ If directions were successfully generated, send to Firebase
    if result.get("status") == "ok":
        send_to_firebase(result["directions"], data["start"], data["end"])

    return jsonify(result)

if __name__ == "__main__":
    app.run(debug=True)
