curl --location --request POST 'localhost:8000/admin/robot' \
--header 'Content-Type: application/json' \
--data-raw '{
    "robot_serial_number": "serial1",
    "assigned" : false,
    "battery_level" : 50
}'

curl --location --request POST 'localhost:8000/user' \
--header 'Content-Type: application/json' \
--data-raw '{
    "user_name":"user1",
    "password":"password1",
    "robot_serial_number": "serial1"
}'

curl --location --request POST 'localhost:8000/auth' \
--header 'Content-Type: application/json' \
--data-raw '{
    "user_name":"user1",
    "password":"password1"
}'

# jwt: eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJ1c2VyX25hbWUiOiJ1c2VyMSIsImV4cCI6MTYxNjU5NTk1Nn0.bUF6dhjEerDMMaDuRANwmbcr1RqMrCjVIG6wt4_xrp0 