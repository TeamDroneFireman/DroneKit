from flask import Flask, url_for, send_file, request
import subprocess
app = Flask(__name__)

@app.route('/')
def root():
    return 'root'

@app.route('/create', methods=['POST'])
def create():
    if request.method == 'POST':
        data = request.json
        instance = data['instance']
        longitude = data['home']['geopoint']['longitude']
        latitude = data['home']['geopoint']['latitude']
        altitude = data['home']['geopoint']['altitude']
        subprocess.Popen(['/home/adra/.local/bin/dronekit-sitl',  'copter',  '--home=' + str(latitude) + ','
        + str(longitude) + ',' + str(altitude) + ',1 ', '--instance=' + str(instance)])
        return 'OK', 200
    return 'Error', 404

@app.route('/mission', methods=['POST'])
def mission():
    if request.method == 'POST':
        data = request.json
        ident = data['id']
        intervention = data['intervention']
        instance = data['instance']
        print str(ident)
        print str(intervention)
        print str(instance)
        print data['mission']['mission']
        geopoints = data['mission']['mission']['geopoints']
        mission = ''
        argg = ['python', 'mission.py', '--id', str(ident),
        '--intervention', str(intervention), '--instance', str(instance),
        '--points']
        for point in range(len(geopoints)):
            """
            argg.append(geopoints[point]['latitude'])
            argg.append(geopoints[point]['longitude'])
            argg.append(geopoints[point]['altitude'])
            """
            argg.append(str(geopoints[point]['latitude']))
            argg.append(str(geopoints[point]['longitude']))
            argg.append(str(geopoints[point]['altitude']))

        print mission
        subprocess.Popen(argg)
        return 'OK', 200
    return 'Error', 404

"""

mission += str(geopoints[point]['latitude'])
mission += ' '
mission += str(geopoints[point]['longitude'])
mission += ' '
mission += str(geopoints[point]['altitude'])
if point != len(geopoints)-1:
    mission += ' '
"""

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
