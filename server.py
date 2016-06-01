from flask import Flask, url_for, send_file, request
import os
app = Flask(__name__)

@app.route('/create', methods=['POST'])
def create():
    if request.method == 'POST':
        data = request.json
        instance = data['instance']
        longitude = data['home']['geopoint']['longitude']
        latitude = data['home']['geopoint']['latitude']
        altitude = data['home']['geopoint']['altitude']
        os.system('dronekit-sitl copter --home=' + str(latitude) + ','
        + str(longitude) + ',' + str(altitude) + ',1 --instance=' + str(instance))
        return 'OK', 200
    return 'Error', 404

@app.route('/mission', methods=['POST'])
def mission():
    if request.method == 'POST':
        data = request.json
        ident = data['id']
        intervention = data['intervention']
        instance = data['instance']
        geopoints = data['mission']['geopoints']
        mission = ''
        for point in range(len(geopoints)):
            mission += str(geopoints[point]['latitude'])
            mission += ' '
            mission += str(geopoints[point]['longitude'])
            mission += ' '
            mission += str(geopoints[point]['altitude'])
            mission += ' '
        print mission
        os.system('python mission.py --id ' + str(ident)
        + ' --intervention ' + str(intervention) + ' --instance' + str(instance)
        + ' --mission' + str(mission))
        return 'OK', 200
    return 'Error', 404


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
