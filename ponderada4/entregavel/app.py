from flask import Flask, request, Response, render_template
from werkzeug.utils import secure_filename
from ultralytics import YOLO
from shutil import rmtree

from db import db_init, db
from models import Img

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///img.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db_init(app)


@app.route('/')
def hello_world():
    return render_template('index.html')


@app.route('/upload', methods=['POST'])
def upload():
    pic = request.files['pic']
    if not pic:
        return 'Nenhuma foto foi enviada.', 400

    filename = secure_filename(pic.filename)
    mimetype = pic.mimetype
    model = YOLO('./best.pt')
    file = pic
    file = file.save("result.jpg")
    result = model.predict("result.jpg", conf=0.6, save=True)
    predicted = open('./runs/detect/predict/result.jpg', 'r' 'b')

    pic = predicted.read()

    img = Img(img=pic, name=filename, mimetype=mimetype)
    db.session.add(img)
    db.session.commit()

    predicted.close()
    rmtree(path="./runs")
    return 'Imagem Enviada', 200


@app.route('/<int:id>')
def get_img(id):
    img = Img.query.filter_by(id=id).first()
    if not img:
        return 'Imagem nao encontrada', 404

    return Response(img.img, mimetype=img.mimetype)

app.run(port=8000, debug=True)