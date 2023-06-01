from fastapi import FastAPI
from fastapi.responses import FileResponse
import os

app = FastAPI()


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.get("/ship")
def show_ship():

    path = '/code/app/res/ship.jpg'

    return FileResponse(path)


@app.get("/rocket_schema")
def show_rocket_schema():

    path = '/code/app/res/rocket_schema.png'

    return FileResponse(path)
