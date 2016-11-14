---
layout: post
title:  "Proyecto: Iteración 1"
date:   2016-10-06 15:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# Proyecto: Iteración 1

## Introducción
Como hemos visto en entradas anteriores, nuestro proyecto consiste en realizar mejoras a la línea de comando de ROS dado que, en el poco tiempo que hemos trabajado con ella, hemos observado varias tareas que podrían ser mejor facilitadas al usuario, algunos errores (bugs) que impiden usar debidamente los comandos y en general una serie de procesos cuya lógica creemos que debería ser modificada para ajustarse a la perspectiva de lo que espera el usuario novato.
Este blog se presenta en este contexto, donde nos damos un espacio para explicar los avances realizados en lo que denominamos la primera iteración (de dos) además de contar nuestras experiencias trabajando en el código interno de ROS.
Dividiremos el blog en cinco partes (incluyendo esta) donde trataremos abiertamente los tópicos que compenden tanto al problema/solución a nivel teórico como a una breve discusión acerca de la implementación práctica, y además hablaremos un poco acerca de cómo queremos hacer llegar nuestras ideas para una posible integración con la versión oficial de ROS.

## Problema
Nuestro proyecto en realidad ataca varios problemas pequeños y localizados en lugar de intentar solucionar uno demasiado grande. Este podría ser descrito de forma general con la siguiente frase: "La línea de comandos que ofrece ROS presenta un conjunto de procedimientos sin la suficiente funcionalidad para adaptarse a nuevos usuarios", a lo que además agregamos que existen errores en ciertos comandos y lógicas que son indispensables pero que no están implementadas. 

Se procederá a describir cada sub-problema de modo que para efectos de este blog, las siguientes secciones se referirán a ellos por su número asignados. Además nos hemos encargado de ordenarlos en orden de menor a mayor dificultad.

1. Obtener información extra de un elemento en ROS se hace con el comando ros--- info, sin embargo rosmsg info no existe. En su lugar se nos propone rosmsg show.

2. La opción que un usuario espera en rostopic pub por default es -1, pero el default actúa diferente de esta.

3. El comando rostopic pub nos pide escribir cada vez el tipo de mensaje, pero este es estático sobre el tópico.

4. La función de autocompletar no sirve cuando rostopic pub lleva opciones.

5. Es difícil saber qué tópicos no tienen subscribers o publishers, ya que hay que comparar cada tópico en dos listas distintas (la de tópicos con subscribers y la de tópicos con publishers).

6. No se puede buscar tipos de mensajes dentro de otros tipos de mensajes, como por ejemplo buscar un Float64 dentro de mensajes de tipo Twist.

7. Al cerrar roscores los tópicos asociados a este no mueren, lo que puede presentar fallas en la lógica esperada si se vuelve a abrir roscore y se hace referencia a tópicos de la sesión anterior.

8. No existe ninguna manera de añadir alguna explicación al momento de crear nodos y tópicos que permita entender al usuario el objetivo de estos. En su lugar se debe deducir la funcionalidad solamente del nombre.


## Solución, diseño e implementación

### Problema 1: No existe rosmsg info
#### Solución
Se propone ...
#### Diseño

#### Implementación