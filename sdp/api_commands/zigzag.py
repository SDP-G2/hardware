import time
import json
import requests

unix_time = int(time.time())

message = {
    "robot_serial_number":"serial1", 
    "time_issued": unix_time, 
    "time_instruction": [unix_time], 
    "instruction": {
        "Task": "ZigZag"
    }
}

headers = {
    'Authorization': 'eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJ1c2VyX25hbWUiOiJ1c2VyMSIsImV4cCI6MTYxNjU5NTk1Nn0.bUF6dhjEerDMMaDuRANwmbcr1RqMrCjVIG6wt4_xrp0'
}

res = requests.post('http://localhost:8000/command', json=message, headers=headers)
res_json = json.loads(res.text)
print(res_json)