/*
  Botão

  Liga e desliga um LED conectado ao pino 13, quando um botão no pino 2 é acionado.

  Circuito:
  - Led conectado ao pino 13 e ao terra com uma resistência de ao menos 100 ohms.
  - Botão do pino 2 para o 5V com ou sem resistor de pullup de 10K.

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

// Pino do botao
constante(numerico, pinoBotao, 2)

// Pino do led
constante(numerico, pinoLed, 13)

// Variavéis podem mudar. Elas são ideais para guardar valores

// Armazena o estado do botão
variavel(numerico, estadoBotao = 0)

// Configura o arduino
configura
inicio

  // Configura o pino do led como saída
  modoPino(pinoLed, SAIDA)

  // Configura o pino do botão como entrada
  modoPino(pinoBotao, ENTRADA_PULLUP)
  
fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Lê o estado do botão
  estadoBotao = leiaDigital(pinoBotao)

  // Checa se o botao esta ativado
  se estadoBotao == LIGADO entao
  
    // Liga o led com o botao ativado
    escrevaDigital(pinoLed, LIGADO)

  // Se ele não estiver
  senao
  
    // Desliga o led com o botão desativado
    escrevaDigital(pinoLed, DESLIGADO)
  
  fimse
  
fim
