/*
  Esmanecer

  Esmanece um led conectado ao pino 9 de forma suave. Para isso é utilizado um pino
  com PWM que seria um pino digital com a capacidade de gerar sinais analógicos.

  Circuito:
  - Led conectado ao pino 9 e ao terra com uma resistência de ao menos 100 ohms.

  O objetivo deste código é o de apresentar a ideia de usar o arduino com linguagem
  portugol, adaptada através do uso de diversas macros. Mais informações consultar
  documentação do projeto ArduinoLang.

  criado 28 de Fevereiro de 2018
  por Jeferson Lima

  Este exemplo está em dominio público.
  
*/

// Inclui biblioteca para linguagem português com estilo da linguagem portugol
#include <ArduinoLangPT_Portugol.h>

// Constantes não mudam. Elas são ideais para informar pinos

// Pino do led
constante(numerico, pinoLed, 9)

// Configura o arduino
configura
inicio

  // Saidas analógicas (PWM), não precisam ser inicializadas
  
fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Declara uma variavél para ser utilizada no laco
  variavel(numerico, valorEsmanecer)

  // Esmanecer saindo de 0 até 255 em passos de 5
  para valorEsmanecer de 0 ate(valorEsmanecer, 255, 5) faca
  inicio
    
    // Escreva o valor do esmanecer de 0 até 255 em passos de 5
    escrevaAnalogico(pinoLed, valorEsmanecer)

    // Espera por 30 milissegundos
    espera(30)
    
  fim

  // Desvanecer saindo de 255 ate 0 em passos de -5
  para valorEsmanecer de 255 ateDecrescente(valorEsmanecer, 0, 5) faca
  inicio

    // Escreva o valor do esmanecer de 255 até 0 em passos de -5
    escrevaAnalogico(pinoLed, valorEsmanecer)

    // Espera por 30 milissegundos
    espera(30)
    
  fim
  
fim
