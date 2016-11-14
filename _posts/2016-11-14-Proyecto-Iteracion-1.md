---
layout: post
title:  "Proyecto: Iteración 1"
date:   2016-11-14 14:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# Proyecto: Iteración 1

## Introducción
Como hemos visto en entradas anteriores, nuestro proyecto consiste en realizar mejoras a la línea de comando de ROS dado que, en el poco tiempo que hemos trabajado con ella, hemos observado varias tareas que podrían ser mejor facilitadas al usuario. 

Además existen algunos errores (bugs) que impiden usar debidamente los comandos y en general una serie de procesos cuya lógica creemos que debería ser modificada para ajustarse a la perspectiva de lo que espera un usuario novato.

Este blog se presenta en este contexto, donde nos damos un espacio para explicar los avances realizados en lo que denominamos la primera iteración (de dos) y donde además contamos nuestras experiencias trabajando en el código interno de ROS.

Dividiremos el blog en cuatro partes (incluyendo esta) donde trataremos abiertamente los tópicos que compenden al problema a nivel teórico y además hablaremos un poco acerca de cómo queremos hacer llegar nuestras ideas para una posible integración con la versión oficial de ROS.

Añadido a esto incluiremos una sección por cada parte de la solución del proyecto donde discutiremos sobre el diseño de esta y, brevemente, sobre la implementación práctica.

Se puede observar el código fuente [aquí](https://github.com/javierdiazp/myros).
