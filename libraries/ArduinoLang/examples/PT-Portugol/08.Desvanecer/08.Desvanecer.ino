/*
  Desvanecer em Portugol

  Este exemplo mostra como desvanecer um led usando a função escreva analógico.

  O objetivo deste código é o de apresentar a ideia de usar o arduino com linguagem
  portugol, adaptada através do uso de diversas macros. Mais informações consultar
  documentação do projeto ArduinoLang.

  criado 28 de Fevereiro de 2018
  por Jeferson Lima

  Este exemplo está em dominio público.
  
*/

// Inclui biblioteca para linguagem português com estilo da linguagem portugol
#include <ArduinoLangPT_Portugol.h>

// Define o pino do led com o valor 9
variavel(numerico, led = 9)

// Define a claridade atual com valor 0
variavel(numerico, luminosidade = 0)

// Define quantos pontos a serem removidos ou adicionadoos da claridade em 5
variavel(numerico, quantidadeDesvanecer = 5)

// Configura o arduino
configura
inicio

  // Configura o pino 9 como saída
  modoPino(led, SAIDA)

fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Configura a luminosidade do pino 9
  escrevaAnalogico(led, luminosidade)

  // Modifique a quantidade de luminosidade atual
  luminosidade = luminosidade + quantidadeDesvanecer

  // Inverte a direcao do desvanecer, quando chegar no limite
  se luminosidade <= 0 ou luminosidade >= 255 entao
    quantidadeDesvanecer = -quantidadeDesvanecer
  fimse

  // Espera por 30 milissegundos para sentir o efeito do escurecimento devagar
  espera(30)

fim
