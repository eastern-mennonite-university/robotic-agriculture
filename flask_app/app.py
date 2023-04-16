from flask import Flask, render_template
from flask_mqtt import Mqtt
import json

app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = 'broker.hivemq.com'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_KEEPALIVE'] = 5

mqtt = Mqtt(app)

last_update = None

@mqtt.on_connect()
def handle_connect(client, userdata, flags, rc):
    mqtt.subscribe('emuagrobot22802/botdata')

@mqtt.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
    data = json.loads(data['payload'])
    print(data)

# Route to display the latest message
@app.route('/')
def index():
    return render_template('index.html', latest_message='hello world')

if __name__ == '__main__':
    app.run(debug=True)