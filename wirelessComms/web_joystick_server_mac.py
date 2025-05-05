from flask import Flask, render_template
import socket

app = Flask(__name__)

jetson_ip = '10.5.144.120'
jetson_port = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send/<v>/<w>') 
def send_command(v, w):
    v = float(v)
    w = float(w)
    command = f"!{v:.2f}@{w:.2f}#"
    print(f"Sending command: {command}")
    sock.sendto(command.encode('utf-8'), (jetson_ip, jetson_port))
    return 'OK', 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

