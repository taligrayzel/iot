from flask import Flask, render_template, request, jsonify
from logic.main import solve_maze_from_web

app = Flask(__name__)

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
    return jsonify(result)

if __name__ == "__main__":
    app.run(debug=True)
