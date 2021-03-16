# curl --location --request POST 'localhost:8000/user' \
# --header 'Content-Type: application/json' \
# --data-raw '{
#     "user_name":"user1",
#     "password":"password1",
#     "robot_serial_number": "serial1"
# }'

curl --location --request POST 'localhost:8000/auth' \
--header 'Content-Type: application/json' \
--data-raw '{
    "user_name":"user1",
    "password":"password1"
}'

# jwt: eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJ1c2VyX25hbWUiOiJ1c2VyMSIsImV4cCI6MTYxNTcxNDQzOX0.E9awdjjhu2RHvh1GWTY5Ek3Zz9q4CHdnkSdPr1jvfdo 