from django.urls import path
from . import views

urlpatterns = [
    path('get_traffic_light_status/', views.get_traffic_light_status),
    path('light_status/', views.light_status),

]
