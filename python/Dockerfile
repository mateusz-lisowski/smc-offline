FROM python:3.9

COPY requirements.txt .

RUN pip install -r requirements.txt

COPY src/ .

COPY dialect/rocket.xml /usr/local/lib/python3.9/site-packages/pymavlink/message_definitions/v1.0

COPY dialect/rocket.xml /usr/local/lib/python3.9/site-packages/pymavlink/dialects/v10

COPY dialect/rocket.py /usr/local/lib/python3.9/site-packages/pymavlink/dialects/v10

CMD ["python", "-u", "./main.py"]