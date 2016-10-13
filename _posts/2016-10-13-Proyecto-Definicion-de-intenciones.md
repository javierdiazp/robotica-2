---
layout: post
title:  "Proyecto: Definición de intenciones"
date:   2016-10-13 17:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# Proyecto: Definición de intenciones

En el contexto de realizar aportes que mejoren la usabilidad del software ROS para manejo de robots proponemos un proyecto que consiste en realizar una serie de mejoras a los comandos en terminal.

## Valor por defecto para las repeticiones de `rostopic pub`
El comportamiento entregado por `rostopic pub` al no indicar un valor de repeticiones o una frecuencia, actúa por defecto publicando una vez el mensaje. Sin embargo el comportamiento difiere respecto a indicar explícitamente el valor 1 para el número de repeticiones dado que este cierra la ejecución tras mandar el mensaje, pero el primero espera a ser cerrado por el usuario.

Nosotros creemos que este comportamiento no es consistente, por lo tanto queremos solucionar este problema definiendo un proceso igual para ambas opciones e implementarlo.

## Comando `rostopic find` opcionalmente recursivo
Cuando el usuario busca todos los tópicos que manejan cierto tipo de mensaje, utiliza el comando `rostopic find`. El problema surge cuando uno de estos tipos está contenido dentro de otro, ya que `rostopic find` solo busca en el top-level de tipos de mensaje.

Para mejorar esto queremos implementar una nueva opción que permita al usuario realizar una búsqueda en los tipos de mensaje de manera que acceda al top-level y a todos los sub-tipos recursivamente.

## Help para tópicos y nodos
Al utilizar los comandos `rostopic info` y `rosnode info` solo se recibe información técnica de las estructuras, pero el creador de estas no tiene manera de comunicar comentarios respecto a su funcionamiento.

Queremos permitir que opcionalmente se agregue un comentario con una breve explicación al definir nodos y crear tópicos, que luego apareza al usar `rostopic info` y `rosnode info`.

## Autocompletar de `rostopic pub`
Cuando se utiliza el comando `rostopic pub` con opciones, la función de autocompletar con tab se encuentra deshabilitada.

Nuestro objetivo es habilitar esta opción, de la misma manera en la que funciona cuando se utiliza el comando sin opciones.

## Tipo redundante en `rostopic pub`
Al utilizar `rostopic pub` es necesario definir el tipo de mensaje que queremos publicar, sin embargo en ROS estos tipos son estáticos respecto del nodo al que nos referimos, por lo que estamos declarando información redundante si el tópico ya había sido definido previamente.

Para solucionar esto queremos permitir que `rostopic pub` sea ejecutado sin declarar el tipo de mensaje, si es que el tópico ya existe, ya que los tópicos son tipados estáticamente.

## Implementar `rosmsg info`
El comando esperado para mostrar información acerca de los tipos de mensajes es `rosmsg info`, como en el resto de comandos similares (rostopic, rosnode, etc). Sin embargo, `rosmsg info` no existe, y el comando que debe usarse en su lugar es `rosmsg show`, lo que rompe la consistencia con los demás comandos.

Para solucionar esto queremos permitir el uso de `rosmsg info` como alias de `rosmsg show`.

## Mostrar lista con tópicos zombie
Cuando un tópico no tiene Publishers, o bien, no tiene Subscribers, no realiza trabajo útil más allá de reservar su tipo (los llamaremos tópicos zombie). Si queremos identificar estos tópicos debemos revisar en la lista entregada por `rostopic list -v` comparando la lista de Publishers y Subscribers o con usando `rostopic info` sobre cada tópico, lo que evidentemente no es práctico.

Queremos implementar una opción `-z` para `rostopic list` que permita encontrar los tópicos zombie.

## Unificar listas de `rostopic list -v`
Cuando se utiliza `rostopic list -v` aparecen dos listas, una que contiene los tópicos publicados y otra con los tópicos suscritos, lo que lleva a tener mucha información duplicada.

Queremos añadir una nueva opción para `rostopic list` que permita visualizar ambas listas de manera unificada, mostrando el nombre del tópico, el tipo de mensaje, la cantidad de subscribers y la cantidad de publishers.

## Manejar zombies al cerrar roscore
Cuando un usuario cierra `roscore`, `rostopic pub` y `rostopic echo` siguen funcionando. Luego, si se vuelve a ejecutar `roscore` los tópicos relacionados a estos comandos son inaccesibles.

Si hay tópicos zombies que están asociados a una instancia de `roscore` que ha sido cerrada, estos tópicos deberían comunicarle a sus subscribers o publishers que ya no es posibles intercambiar información a través de estos tópicos .
