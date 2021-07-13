# -*- coding: utf-8 -*-
from flask import Flask, jsonify, request, send_file, redirect
from flask_cors import CORS
import glob
import roslibpy
import os
import subprocess
import time
import shutil

from check_gps_fix import *

# The code version for the CAP embedded software
# Update here to let the user know in the app
# some change was made in the Update process
firmware_version = '1.2.0'

# Method to find the space used in a folder
# Will be used to get available memory to work with
def get_size(start_path='.'):
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(start_path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            # skip if it is symbolic link
            if not os.path.islink(fp):
                total_size += os.path.getsize(fp)

    return total_size

global_ros_ip = '0.0.0.0'
global_ros_port = 9090
ros = roslibpy.Ros(host=global_ros_ip, port=global_ros_port)
ros.on_ready(lambda: print('Is ROS connected ?', ros.is_connected))
ros.run()

global_aquisition_type = 0
global_project_root_path = '/home/cap/Desktop/'
global_project_accepted_types = ['ambientes', 'objetos']
global_feedback = ''
global_feedback_topic = roslibpy.Topic(ros, '/feedback_scan', 'std_msgs/Float32')
global_image_camera = ''
global_image_topic = roslibpy.Topic(ros, '/image_user/compressed', 'sensor_msgs/CompressedImage')
global_camera_state = False
global_camera_state_topic = roslibpy.Topic(ros, '/camera_state', 'std_msgs/Bool')
global_scan_state = False
global_scan_state_topic = roslibpy.Topic(ros, '/scan_state', 'std_msgs/Bool')

app = Flask(__name__)
cors = CORS(app)


@app.route("/")
def get_default_route():
    return redirect('/project/list')


@app.route("/acquisition/cancel", methods=['POST'])
def post_acquisition_cancel():
    global global_camera_state
    global global_scan_state
    global global_feedback
    global global_image_camera

    os.system('rosnode kill camera')
    os.system('rosnode kill imagem_lr_app')
    os.system('rosnode kill livox_lidar_publisher')
    os.system('rosnode kill multi_port_cap')
    os.system('rosnode kill scanner_space')
    os.system('rosnode kill acc_space_node')
    os.system('rosnode kill scanner_obj')
    os.system('rosnode kill gps_node')

    camera_force_kill()

    return jsonify('ok')


@app.route("/acquisition/capture_obj", methods=['POST'])
def post_acquisition_capture_obj():
    process = subprocess.Popen('rosservice call /capturar_obj 1', shell=True, stdout=subprocess.PIPE)
    process.wait()
    return jsonify(True)


@app.route("/acquisition/finish_capture_obj", methods=['POST'])
def post_acquisition_finish_capture_obj():
    process = subprocess.Popen('rosservice call /capturar_obj 2', shell=True, stdout=subprocess.PIPE)
    process.wait()
    camera_stop()
    return jsonify(True)


@app.route("/acquisition/type", methods=['GET'])
def get_acquisition_type():
    global global_aquisition_type
    return jsonify(global_aquisition_type)


@app.route("/camera/is_busy", methods=['GET'])
def get_camera_busy():
    global global_camera_state
    global global_scan_state
    is_busy = False

    if global_camera_state and global_scan_state:
        is_busy = True

    return jsonify(is_busy)


@app.route('/camera/image', methods=['GET'])
def get_camera_image():
    global global_image_camera
    return global_image_camera


@app.route("/camera/force_kill", methods=['POST'])
def camera_force_kill():
    global global_camera_state
    global global_scan_state
    global global_feedback
    global global_image_camera

    process = subprocess.Popen('fuser -k /dev/video0', shell=True, stdout=subprocess.PIPE)
    process.wait()

    global_camera_state = False
    global_scan_state = False
    global_feedback = 0.0
    global_image_camera = ''

    return jsonify(True)


@app.route("/camera/kill", methods=['POST'])
def camera_kill():
    global global_image_camera
    os.system('rosnode kill imagem_lr_app')
    process = subprocess.Popen('rosnode kill camera', shell=True, stdout=subprocess.PIPE)
    process.wait()
    global_image_camera = ''
    return jsonify(True)


@app.route("/camera/start", methods=['POST'])
def camera_start():
    global global_camera_state
    global global_scan_state

    if not global_camera_state and not global_scan_state:
        process = subprocess.Popen('roslaunch cv_camera calibrar_camera.launch', shell=True, stdout=subprocess.PIPE)
        process.wait()

    return jsonify(global_scan_state)


@app.route("/camera/stop", methods=['POST'])
def camera_stop():
    global global_camera_state
    global global_scan_state

    print('global_camera_state')
    print(global_camera_state)
    print('global_scan_state')
    print(global_scan_state)

    if global_camera_state and not global_scan_state:
        os.system('rosnode kill multi_port_cap')
        os.system('rosnode kill send_dynamixel_to_zero')
        camera_kill()

    return jsonify(True)


@app.route("/camera/feedback", methods=['GET'])
def get_feedback():
    global global_feedback
    return jsonify(global_feedback)


@app.route("/date", methods=['GET'])
def get_date():
    process = subprocess.Popen('date', stdout=subprocess.PIPE)
    out, err = process.communicate(timeout=2)

    return jsonify(out)


@app.route("/date", methods=['POST'])
def post_date():
    data = request.get_json()
    param_date = data['dateStr']
    os.system("echo 12 | sudo -S timedatectl set-ntp 0 && echo 12 | sudo -S timedatectl set-time '" + str(param_date) + "' && echo 12 | sudo -S hwclock -w")

    return jsonify(True)


@app.route("/project/delete", methods=['POST'])
def project_delete():
    data = request.get_json()
    param_arquivo = data['arquivoStr']
    param_nome = data['nomeStr']
    param_scan = data['scanStr']
    param_tipo = data['tipo']

    try:
        if param_tipo in global_project_accepted_types:
            project_path = os.path.join(global_project_root_path, param_tipo)

            if param_nome and param_scan and param_arquivo:
                os.remove(os.path.join(project_path, param_nome, param_scan, param_arquivo))
            if param_nome and param_scan:
                shutil.rmtree(os.path.join(project_path, param_nome, param_scan))
            elif param_nome:
                shutil.rmtree(os.path.join(project_path, param_nome))
    except:
        msg = f'Não foi possível excluir: {param_tipo}/{param_nome}/{param_scan}/{param_arquivo}'
        print(msg)

    return jsonify('ok')


@app.route("/project/list", methods=['GET'])
def project_list():
    d = []

    for folder in glob.iglob(os.path.join(global_project_root_path, '*/*'), recursive=False):
        try:
            folder_data = {
                'nomeStr': os.path.basename(folder),
                'tipo': os.path.basename(os.path.dirname(folder)),
                'criacaoDate': os.path.getctime(folder),
                'modificacaoDate': os.path.getmtime(folder)
            }

            if folder_data['tipo'] in global_project_accepted_types:
                d.append(folder_data)
        except:
            print('Não foi possível ler os dados do projeto: ' + folder)

    return jsonify(d)


@app.route("/project/<project_type>/<project_name>", methods=['GET'])
def project(project_type, project_name):
    d = []

    for folder in os.scandir(os.path.join(global_project_root_path, project_type, project_name)):
        for root, dirs, files in os.walk(folder):
            scan = {
                'nomeStr': folder.name,
                'criacaoDate': os.path.getctime(folder.path),
                'modificacaoDate': os.path.getmtime(folder.path)
            }

            scan_files = []

            for f in files:
                try:
                    file_path = os.path.join(folder.path, f)
                    file_data = {
                        'nomeStr': os.path.basename(file_path),
                        'criacaoDate': os.path.getctime(file_path),
                        'modificacaoDate': os.path.getmtime(file_path)
                    }

                    if project_type in global_project_accepted_types:
                        scan_files.append(file_data)
                except:
                    print('Não foi possível ler os dados do arquivo: ' + f)

            scan['fileList'] = scan_files
            d.append(scan)

    return jsonify(d)


@app.route('/project/<projecttype>/<projectname>/<scan>/<filename>', methods=['GET'])
def project_download_file(projecttype, projectname, scan, filename):
    return send_file(os.path.join(global_project_root_path, projecttype, projectname, scan, filename),
                     as_attachment=True)


@app.route("/project/acquisition", methods=['POST'])
def project_new():
    global global_feedback
    global global_aquisition_type
    data = request.get_json()
    nome_str = data['nomeStr']
    tipo_int = data['tipoInt']

    camera_stop()

    if tipo_int == 0:
        time.sleep(10)
        subprocess.Popen([f'roslaunch pepo_space pepo_space.launch pasta:={nome_str}'], shell=True)
    else:
        subprocess.Popen([f'roslaunch pepo_obj pepo_obj.launch pasta:={nome_str}'], shell=True)

    global_aquisition_type = tipo_int
    return jsonify(True)

@app.route("/gps/check_fix", methods=["POST"])
def check_fix():
    return jsonify(check())

@app.route("/firmware_version", methods=["GET"])
def get_firmware_version():
    global firmware_version
    return jsonify(firmware_version)

@app.route('/ping', methods=['GET'])
def ping():
    return jsonify(time.time())

# Ros
@app.route('/ros/status', methods=['GET'])
def ros_status():
    st = os.statvfs('/')
    free = st.f_bavail * st.f_frsize
    used = get_size('/home/cap/Desktop/')
    total = free + used
    used_pct = 100.0 * float(used) / float(total) if 100.0 * float(used) / float(total) > 1 else 0.5

    print('used: %.2f total: %.2f free: %.2f used_pct: (%.2f)'%(used, total, free, used_pct))

    data = {
        'is_ros_connected_bool': ros.is_connected,
        'distro_str': roslibpy.Param(ros, 'rosdistro').get(callback=None, timeout=30).rstrip().capitalize(),
        'version_str': roslibpy.Param(ros, 'rosversion').get(callback=None, timeout=30).rstrip(),
        'free': round(free, 2),
        'total': round(total, 2),
        'used': round(used, 2),
        'used_percent': round(used_pct, 2)
    }

    return jsonify(data)


@app.route('/ros/get_param/<param_name>', methods=['GET'])
def ros_get_param(param_name):
    value = roslibpy.Param(ros, param_name).get(callback=None, timeout=30)
    return jsonify(value)


@app.route('/ros/set_param', methods=['POST'])
def ros_set_param():
    data = request.get_json()
    param = data['param']
    value = data['value']
    roslibpy.Param(ros, param).set(value, callback=None, timeout=30)

    # X=1(desligado) ou X=3(ligado)
    if param == 'exposure_auto':
        os.system('v4l2-ctl --set-ctrl=exposure_auto=' + str(value))

    # Range 0 < X < 2500
    elif param == 'exposure_absolute':
        is_exposure_auto = roslibpy.Param(ros, 'exposure_auto').get(callback=None, timeout=30)

        if 0 <= value <= 2500 and is_exposure_auto == 1:
            os.system('v4l2-ctl --set-ctrl=exposure_absolute=' + str(value))

    # Range 0 < X < 200
    elif param == 'brightness':
        if 0 <= value <= 200:
            os.system('v4l2-ctl --set-ctrl=brightness=' + str(value))

    # Range 50 < X < 200
    elif param == 'saturation':
        if 50 <= value <= 200:
            os.system('v4l2-ctl --set-ctrl=saturation=' + str(value))

    # Undefined
    elif param == 'contrast':
        os.system('v4l2-ctl --set-ctrl=contrast=' + str(value))

    # X=0(desligado) ou X=1(ligado)
    elif param == 'white_balance_temperature_auto':
        os.system('v4l2-ctl --set-ctrl=white_balance_temperature_auto=' + str(value))

    # Range 2800 < X < 6000
    elif param == 'white_balance_temperature':
        roslibpy.Param(ros, 'white_balance_temperature_auto').get(callback=None, timeout=30)

        if 2800 <= value <= 6000:
            os.system('v4l2-ctl --set-ctrl=white_balance_temperature=' + str(value))

    return jsonify(value)


def camera_image_callback(msg):
    global global_image_camera
    global_image_camera = msg['data']


def camera_state_callback(msg):
    global global_camera_state
    global_camera_state = msg['data']


def feedback_callback(msg):
    global global_feedback
    global_feedback = msg['data']


def scan_state_callback(msg):
    global global_scan_state
    global_scan_state = msg['data']


def init_subscribers():
    global global_camera_state_topic
    global global_feedback_topic
    global global_image_topic
    global global_scan_state_topic

    global_image_topic.subscribe(callback=camera_image_callback)
    global_camera_state_topic.subscribe(callback=camera_state_callback)
    global_feedback_topic.subscribe(callback=feedback_callback)
    global_scan_state_topic.subscribe(callback=scan_state_callback)


init_subscribers()

if __name__ == '__main__':
    app.run()
