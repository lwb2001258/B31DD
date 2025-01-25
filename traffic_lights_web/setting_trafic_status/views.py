import random
from django.shortcuts import render, redirect
from django.http import JsonResponse

import time
import redis

# ser = serial.Serial('COM7',9600)
# time.sleep(2)
r = redis.Redis(host='localhost', port=6379, decode_responses=True)


def get_traffic_light_status(request):
    ratio = float(r.get("ratio"))
    stable_energy = 1.8*15 + 5.0*2.0 + 5*0.5
    variable_energy = 5.0 * 15 + 1.2*15

    total_energy =stable_energy+ variable_energy * ratio
    emergency_status = r.get("EMERGENCY")
    if emergency_status == "NO-GREEN-0-RED-0":
        light_status = r.get("light_status")
        color = light_status.split("-")[0]
        _time_stamp = int(light_status.split("-")[1])
        time_stamp = int(time.time())
        if color == "GREEN":
            time_counts = 61-(time_stamp - _time_stamp)
            if time_counts < 0:
                color = "YELLOW"
                time_counts = 0
        else:
            time_counts = 31 - (time_stamp - _time_stamp)
        return JsonResponse({"color": color, "time": time_counts,"total_energy": total_energy})
    else:
        light_status = r.get("light_status")
        _time_stamp = 0
        if light_status.startswith("EMERGENCY"):
            _time_stamp = int(light_status.split("-")[-1])
        time_stamp = int(time.time())
        if (time_stamp - _time_stamp) <60:
            return JsonResponse({"color": "GREEN", "time": "EMERGENCY","total_energy": total_energy})
        time_diff = time_stamp - _time_stamp - 60
        color = "GREEN"
        time_counts = 0
        green_tag = int(emergency_status.split("-")[2])
        red_tag = int(emergency_status.split("-")[-1])
        if green_tag:
            color = "GREEN"
            time_counts = green_tag-time_diff+2
        elif red_tag:
            color = "RED"
            time_counts = red_tag-time_diff+2

        return JsonResponse({"color": color, "time": time_counts,"total_energy": total_energy})




def light_status(request):
    return render(request, "setting_trafic_status/light_status.html")


