# Mirobot Block Classifier

En este repositorio puedes encontrar diferentes archivos para ejecutar un ejemplo de un robot clasificador de bloques de colores utilizando el Mirobot en el simulador de Coppelia Sim junto con su [API remota](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)

# Estructura

```
./
|-- scene
    |-- color-classification.ttt
|-- remote
	|-- __init__.py
    |-- coppelia.py
    |-- sim.py
    |-- simConst.py
    |-- remoteApi.dll
|-- main.py
```

1. `scene` es una carpeta que tiene la escena utilizada para el ejemplo de clasificación. Las escenas son archivos utilizados por el simulador, los cuales contienen todos los componentes, robots, actuadores y sensores utilizados. La escena se llamada `color-classification.ttt`.
2. `remote` contiene todo el código utilizado para el control de los sensores, actuadores y el robot del simulador de coppelia. Dentro de esta carpeta puedes encontrar diferentes archivos
   - `__init__.py` es un archivo que le indica a python que la carpeta es un modulo
   - `coppelia.py` contiene 6 clases utilizadas para el uso de los robots, actuadores y sensores dentro del simulador. Las clases son las siguientes:
     - `Client` es una clase utilizada para inicializar la conexión entre python y el simulador de coppelia. La misma contiene un método estático que se encarga de ejecutar la conexión
     - `Object` es una clase que permite conectarnos con objectos dentro del simulador y obtener sus posiciones (si se desea)
     - `Mirobot` es una clase que permite el control del Mirobot. La clase contiene todos los calculos de cinemática inversa y directa junto con las medidas
     - `Camera` es una clase que permite generar una conexión con la camara del simulador. Contiene un método encargado de generar la clasificación de colores
     - `Conveyor` es una clase que permite generar una conexión con la banda transportadora del simulador. Contiene un método que permite controlar la velocidad del dispositivo
     - `ProximitySensor` es una clase que permite generar una conexión con el sensor de proximidad. Contiene un método que permite leer data del sensor de proximidad
   - `sim.py`, `simConst.py` y `remoteApi.dll` son archivos que contienen que permiten generar una conexión entre el simulador y python. Revisar [acá](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) para más información
3. `main.py` es el código principal utilizado para realizar la simulación. Si deseas realizar el ejemplo, revisa las siguientes intrucciones.

# Simulación

Para ejecutar este ejemplo, sigue los siguientes pasos:

1. Antes que nada, asegurate de tener Python 3 instalado en tu sistema, puedes descargar python en el siguiente [enlace](https://python.org/downloads/). Se recomienda en la instalación marcar la casilla que añade python a PATH
2. También debes tener git instalado en tu sistema. Puedes instalar git mediante el siguiente [enlace](https://git-scm.com/)
3. Clona este repositorio en tu terminal utilizando el siguiente comando:

```
git clone https://github.com/Jrigoo/Blocks-Classifier .
```

4. Se recomienda utilizar un ambiente virtual para descargar las dependencias. Para crear un ambiente virtual,ejecuta el siguiente comando en tu terminal:

```
python -m venv env
```

5. Accede a tu ambiente virtual. ejecuta el siguiente comando en tu terminal dependiendo de tu tipo de terminal

```
# Powershell/CMD
./env/Scripts/activate

# Bash
source env/Scripts/activate
```

4. Descarga las dependencias necesarias para el proyecto dentro de tu ambiente virtual. ejecuta el siguiente comando en tu terminal:

```
pip install -r requirements.txt
```

Esto te permitirá descargar todas las dependencias necesarias para este proyecto en un ambiente aislado. 4. Una vez descargadas las dependencias, ubica el archivo `color-classification.ttt` de la carpeta `scene` 5. Abre el simulador de Coppelia Sim (puedes descargarlo en la [página oficial](https://coppeliarobotics.com/)),abre el archivo `color-classification.ttt` cómo una nueva scene o escena y empieza la simulación 6. Una vez empezada la simulación, ve al archivo `main.py` y ejecuta el código de python desde tu terminal:

```
python main.py
```

7. Una vez ejecutado el código, puedes revisar la simulación en proceso. Para detener la simulación marca el botón de detener en el simulador y en la terminal presiona ctrl + c para generar una interrupción
