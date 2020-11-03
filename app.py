# -*- coding: utf-8 -*-

from collections import defaultdict
from flask import Flask, jsonify, request, send_file, redirect, url_for, make_response
from flask_cors import CORS
import glob
import json
import roslibpy
import os 
import time
import shutil

projectListPath = '/mnt/dev/capmobilefiles/'
projectTypes = ['ambientes', 'objetos']

app = Flask(__name__)
cors = CORS(app)

#ros = roslibpy.Ros(host='192.168.0.139', port=9090)
ros = roslibpy.Ros(host='0.0.0.0', port=9090)
ros.run()

imageTopic = roslibpy.Topic(ros, '/image_user/compressed', 'sensor_msgs/CompressedImage')
imageCamera = ''

@app.route("/")
def getDefaultRoute():
    return redirect('/project/list')

@app.route("/project/aquisicao", methods=['POST'])
def aquisicaoProject():
    data = request.get_json()
    nomeStr = data['nomeStr']
    tipoInt = data['tipoInt']

    #TODO: Verificar se o projeto já existe
    #TODO: Verificar como será implementado
    
    os.system('roslaunch pepo_space pepo_space.launch pasta:={nomeStr}')  

    return jsonify(True)

@app.route("/project/delete", methods=['DELETE'])
def deleteProject():
    data = request.get_json()
    paramArquivo = data['arquivoStr']
    paramNome = data['nomeStr']
    paramTipo = data['tipo']   

    try:
        if paramTipo in projectTypes:
            if paramNome and paramTipo and paramArquivo:
                os.remove(os.path.join(projectListPath, paramTipo, paramNome, paramArquivo))
            elif paramNome and paramTipo:
                shutil.rmtree(os.path.join(projectListPath, paramTipo, paramNome))

        return make_response(jsonify({}), 204)
    except:
        msg = 'Não foi possível excluir o diretório: ' + paramTipo + '/' + paramNome
        print(msg)
        return make_response(jsonify(msg), 500)

@app.route("/project/list", methods=['GET'])
def getProjectList():
    d = []

    for folder in glob.iglob(os.path.join(projectListPath, '*/*'), recursive=False):
        try:
            folderData = {}
            folderData['nomeStr'] = os.path.basename(folder)
            folderData['tipo'] = os.path.basename(os.path.dirname(folder))
            folderData['criacaoDate'] = os.path.getctime(folder)
            folderData['modificacaoDate'] = os.path.getmtime(folder)

            if folderData['tipo'] in projectTypes:
                d.append(folderData)
        except:
            print('Não foi possível ler os dados do projeto: ' + folder)

    return jsonify(d)

@app.route("/project/list/<projecttype>/<projectname>", methods=['GET'])
def getProject(projecttype, projectname):
    d = []

    for root, dirs, files in os.walk(os.path.join(projectListPath, projecttype, projectname), topdown=False):
       
        for f in files:
            try:
                fileData = {}
                fullpath = os.path.join(projectListPath, projecttype, projectname, f)
                fileData['nomeStr'] = os.path.basename(fullpath)
                fileData['criacaoDate'] = os.path.getctime(fullpath)
                fileData['modificacaoDate'] = os.path.getmtime(fullpath)

                if projecttype in projectTypes:
                    d.append(fileData)
            except:
                print('Não foi possível ler os dados do arquivo: ' + f)
          

    return jsonify(d)

@app.route('/project/list/<projecttype>/<projectname>/<filename>', methods=['GET'])
def downloadFile(projecttype, projectname, filename):
    return send_file(os.path.join(projectListPath, projecttype, projectname,filename) , as_attachment=True)

@app.route('/ping', methods=['GET'])
def ping():
    return jsonify(time.time())

# Ros
@app.route('/ros/status', methods=['GET'])
def rosStatus():
    data = {
        'is_ros_connected_bool': ros.is_connected,
        'distro_str': ros.get_param('rosdistro').rstrip().capitalize(),
        'version_str': ros.get_param('rosversion').rstrip()
    }

    return jsonify(data)

@app.route('/ros/getParam/<paramname>', methods=['GET'])
def rosGetParam(paramname):
    return jsonify(ros.get_param(paramname))     

@app.route('/ros/setParam', methods=['POST'])
def rosSetParam():  
    data = request.get_json()
    param = data['param']
    value = data['value']
    ros.set_param(param, value)

    # X=1(desligado) ou X=3(ligado)
    if param == 'exposure_auto':
        
        commandValue = 1

        if value == True:
            commandValue = 3

        os.system('v4l2-ctl --set-ctrl=exposure_auto=' + str(commandValue))

    # Range 0 < X < 2500
    elif param == 'exposure_absolute':        
        if value >= 0 and value <= 2500:
            os.system('v4l2-ctl --set-ctrl=exposure_absolute=' + str(value))

    # Range 0 < X < 200
    elif param == 'brightness':        
        if value >= 0 and value <= 200:
            os.system('v4l2-ctl --set-ctrl=brightness=' + str(value))        

    # Range 50 < X < 200
    elif param == 'saturation':        
        if value >= 50 and value <= 200:
            os.system('v4l2-ctl --set-ctrl=saturation=' + str(value))

    # Undefined
    elif param == 'contrast':        
            os.system('v4l2-ctl --set-ctrl=contrast=' + str(value))        

    # X=0(desligado) ou X=1(ligado)
    elif param == 'white_balance_temperature_auto':
        
        commandValue = 0

        if value == True:
            commandValue = 1

        os.system('v4l2-ctl --set-ctrl=white_balance_temperature_auto=' + str(commandValue))        

    # Range 2800 < X < 6000
    elif param == 'white_balance_temperature':        
        if value >= 50 and value <= 200:
            os.system('v4l2-ctl --set-ctrl=white_balance_temperature=' + str(value))          

    return jsonify(value)

def setImageCamera(msg):
    global imageCamera 
    imageTopic.unsubscribe()
    imageCamera = msg['data']

@app.route('/camera', methods=['GET'])
def getCamera():
    global imageCamera 
    rosCamera()
    
    return imageCamera

def rosCamera():
    if ros.is_connected:
        imageTopic.subscribe(setImageCamera)

if __name__ == '__main__':
   app.run()  