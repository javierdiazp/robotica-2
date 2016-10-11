---
layout: post
title:  "Proyecto: Definición de intenciones"
date:   2016-10-11 17:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# Proyecto: Definición de intenciones

## Valor por defecto de rostopic pub (-1)
¿Por qué es distinto el comportamiento entre -1 y sin argumento?

## Rostopic find recursivo (como opción)
La idea es poder buscar qué tópicos utilizan mensajes del tipo dado, debajo del top-level

## Help de tópicos y nodos
Permitir que opcionalmente se agregue un comentario con una breve explicación para nodos y tópicos, que apareza al usar ros* info

## Autocompletar de rostopic pub
Corregir el autocompletar con tab cuando se utiliza pub con argumentos opcionales

## Tipo redundante en pub
Al utilizar rostopic pub es necesario definir el tipo de mensaje, cuando este es estático para la definición del tópico.

## Rosmsg usa show en vez de info
Permitir el uso de rosmsg info como alias de rosmsg show para mantener la consistencia con el resto de elementos de ros.