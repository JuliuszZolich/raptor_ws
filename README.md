# Raptor_ws
Repozytorium zawiera projekt ROS, który ma na celu integrację łazika przygotowanego przez zespół Raptors z systemem ROS. 
Projekt składa się z pakietów:
- can_wrapper - pakiet odpowiedzialny za komunikację z układem CAN
- mqtt_bridge - pakiet odpowiedzialny za komunikację z brokerem MQTT
- ros_can_integration - pakiet odpowiedzialny za integrację komunikacji CAN z ROS
- quad_rover_kinematics - zewnętrzna biblioteka odpowiedzialna za kinematykę łazika
- libVescCan - zewnętrzna biblioteka odpowiedzialna za komunikację z układem VESC
- .devcontainer - konfiguracja kontenera deweloperskiego
- .vscode - konfiguracja środowiska Visual Studio Code

## Wymagania

### Wymagane oprogramowanie
- Docker
- Docker Compose
- Visual Studio Code

## Uruchomienie projektu
1. Sklonuj repozytorium
2. Uruchom Visual Studio Code
3. Otwórz folder raptor_ws
4. Wciśnij kombinację klawiszy `Ctrl + Shift + P`
5. Wpisz `Remote-Containers: Reopen in Container` i wybierz opcję
6. Po zakończeniu procesu budowania kontenera, otwórz terminal w Visual Studio Code
7. Z listy konfiguracji wybierz ROS:roslaunch

## Struktura projektu
```asm
raptor_ws/ 
├── .devcontainer/ 
│ ├── devcontainer.json 
│ ├── Dockerfile 
│ ├── mosquitto.conf 
│ ├── rex 
│ └── rexlaunch.sh 
├── .vscode/ │ 
├── c_cpp_properties.json 
│ ├── extensions.json 
│ ├── launch.json 
│ ├── settings.json 
│ └── tasks.json 
├── src/ 
│ ├── can_wrapper/ 
│ │ ├── include/can_wrapper/
│ │ │ └── Pliki_nagłówkowe_obsługujące_komunikację_CAN (.HPP)
│ │ ├── launch/ 
│ │ │ └── can_wrapper.launch 
│ │ ├── msg/
│ │ │ └── Pliki_wiadomości (.MSG)
│ │ ├── src/
│ │ │ └── Pliki_źródłowe_obsługujące_komunikację_CAN (.CPP)
│ │ ├── CMakeLists.txt 
│ │ └── package.xml 
│ ├── libVescCan/ (Zewnętrzna biblioteka)
│ ├── mqtt_bridge/
│ │ ├── include/mqtt_bridge/ 
│ │ │ ├── ROSTopicHandler.hpp 
│ │ │ └── rapidjson (Zewnętrzna biblioteka)
│ │ ├── launch/ 
│ │ │ └── mqtt_bridge.launch 
│ │ ├── msg/
│ │ │ └── ManipulatorMessage.msg 
│ │ ├── src/ 
│ │ │ ├── mqtt_bridge_node.cpp
│ │ │ ├── ROSTopicHandler.cpp 
│ │ │ └── CMakeLists.txt 
│ │ └── package.xml 
│ ├── quad_rover_kinematics/ (Zewnętrzna biblioteka)
│ ├── ros_can_integration/ 
│ │ ├── include/ros_can_integration/ 
│ │ │ └── CanSocket.hpp 
│ │ ├── launch/ 
│ │ │ └── ros_can_integration.launch
│ │ ├── src/ 
│ │ │ ├── CanSocket.cpp
│ │ │ ├── ros_can_integration.cpp 
│ │ │ └── CMakeLists.txt 
│ │ └── package.xml 
├── .catkin_workspace 
├── .gitignore 
├── .gitmodules
├── docker-compose.yml 
├── master.launch 
└── README.md
```