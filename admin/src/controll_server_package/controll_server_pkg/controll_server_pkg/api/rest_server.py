from flask import Flask, jsonify

app = Flask(__name__)

@app.route('/ping')
def ping():
    return jsonify({"message": "pong"})

def run_rest():
    app.run(host='0.0.0.0', port=8000)
