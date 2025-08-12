/*
  Pisca em Portugol

  Liga o led por um segundo desliga o mesmo por um segundo, espera outro segundo
  e assim por diante.

  O objetivo deste código é o de apresentar a ideia de usar o arduino com linguagem
  portugol, adaptada através do uso de diversas macros. Mais informações consultar
  documentação do projeto ArduinoLang.

  criado 28 de Fevereiro de 2018
  por Jeferson Lima

  Este exemplo está em dominio público.
  
*/

// Inclui biblioteca para linguagem português com estilo da linguagem portugol
#include <ArduinoLangPT_Portugol.h>

// Configura o arduino
configura
inicio

  // Configura o pino 13 como saída
  modoPino(LED_INTEGRADO, SAIDA)

fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Liga o Led no Pino 13
  escrevaDigital(LED_INTEGRADO, LIGADO)

  // Espera por um segundo
  espera(UM_SEGUNDO)

  // Desliga o Led no Pino 13
  escrevaDigital(LED_INTEGRADO, DESLIGADO)

  // Espera outro segundo
  espera(UM_SEGUNDO)

fim
