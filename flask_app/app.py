from flask import Flask, render_template, request
from flask_mqtt import Mqtt
import json
from datetime import datetime

app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = 'broker.hivemq.com'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_KEEPALIVE'] = 5

mqtt = Mqtt(app)

bot_data = dict()
bot_data['last_update'] = None

@mqtt.on_connect()
def handle_connect(client, userdata, flags, rc):
    mqtt.subscribe('emuagrobot22802/botdata')

@mqtt.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
    bot_data.update(json.loads(data['payload']))
    bot_data['last_update'] = datetime.now().strftime("%d %b. %H:%M:%S")
    print(data)

# Route to display the latest message
@app.route('/agrobot/')
def index():
    return render_template('index.html', data=bot_data)

@app.route('/agrobot/publish', methods=['POST'])
def publish_message():
    payload = request.form['payload']
    mqtt.publish('emuagrobot22802/control', payload)
    return 'Message published'

if __name__ == '__main__':
    app.run(debug=True)