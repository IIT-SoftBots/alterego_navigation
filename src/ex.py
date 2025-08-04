from time import sleep
import requests
import sys
import time
from colorama import Fore
import inspect
import random

backend_url = "http://192.168.178.80:1984/"
auth_header = ""
headers = {"Authorization": auth_header}

def login():
  global auth_header
  global headers
  response = requests.post(
    backend_url + "login", 
    json={"username": "admin", "password": "admin"},
  )
  if response.ok:
    auth_header = response.content.decode("utf-8")
    headers = {"Authorization": auth_header}
    print("Auth Header: ", auth_header)
    print("Login Success")
    return True
  else:
    print("Status Code: ", response.status_code)
    print("Response Content: ", response.content.decode("utf-8"))
    print("Login Failed")
    return False

def logout():
  global auth_header
  global headers
  result = post_request("logout", {})
  if result:
    auth_header = ""
    headers = {"Authorization": auth_header}
    print("Logout Success")
  else:
    print("Logout Failed")

def get_request(endpoint): 
  response = requests.get(backend_url + endpoint, headers=headers)
  # print_response("GET", endpoint, response)
  return response

def post_request(endpoint, data):
  response = requests.post(backend_url + endpoint, json=data, headers=headers)
  print_response("POST", endpoint, response, data)
  return response

def print_response(request, endpoint, response, data=None):
  print(
    request + " at time [" + str(time.time()) 
    + "] with code [" 
    + str(response.status_code) + "] on " + endpoint + " "
    + (str(data) if data else "") + " : " + response.content.decode("utf-8")
  )

if __name__ == "__main__":

  waypoint_number = 5
  mission_number = 100
  mission_iter = 0

  try:
    if not login():
      sys.exit(1)

    print("\n>> switch to PreNavigation")
    post_request("system-status", {"status": "PreNavigation"})

    system_status = get_request("system-status").json()["content"]["status"]

    while system_status != "PreNavigation":
      sleep(1)
      system_status = get_request("system-status").json()["content"]["status"]

    print("\n>> localize robot in waypoint 0")
    post_request("localize-waypoint", {"map": "navigation_map","waypoint": "0"})
    
    sleep(1)

    while mission_iter < mission_number: 
      number = str(random.randint(0, waypoint_number-1))
      print("\n>> send robot to random waypoint: ",number)
      post_request("mission", {"action": "start-mission", "waypoints": [{"name": f"{number}", "radius": 0.0}],"map": "navigation_map"})

      sleep(1)
      robot_status = get_request("robot-status").json()["content"]["status"]
      while robot_status != "IDLE":
        sleep(1)
        robot_status = get_request("robot-status").json()["content"]["status"]
      mission_iter = mission_iter + 1

    print("\n>> DONE!")

  except Exception as e:
    print("\n !!! Error:", e)
    logout()
    sys.exit(0)
  logout()
