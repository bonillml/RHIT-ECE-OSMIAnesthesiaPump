/*
  Leitura Digital e Serial em Portugol

  Este exemplo mostra como ler o status de um pino digital e enviar o mesmo pela serial.

  O objetivo deste código é o de apresentar a ideia de usar o arduino com linguagem
  portugol, adaptada através do uso de diversas macros. Mais informações consultar
  documentação do projeto ArduinoLang.

  criado 28 de Fevereiro de 2018
  por Jeferson Lima

  Este exemplo está em dominio público.
  
*/

// Inclui biblioteca para linguagem português com estilo da linguagem portugol
#include <ArduinoLangPT_Portugol.h>

// Define o pino do botao como o pino 4
variavel(numerico, botaoMomentaneo = 4)

// Configura o arduino
configura
inicio

  // Inicializa a comunicação serial
  comeca(Serial, 9600)

  // Configura o pino 4 como entrada
  modoPino(botaoMomentaneo, ENTRADA)

fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Le o estado do pino com botao
  variavel(numerico, estadoBotao)
  estadoBotao = leiaDigital(botaoMomentaneo)

  // Imprime o estado do botão na Serial
  imprimeln(Serial, estadoBotao)

  // Espera por 1 milissegundo para aumentar a estabilidade
  espera(1)

fim
