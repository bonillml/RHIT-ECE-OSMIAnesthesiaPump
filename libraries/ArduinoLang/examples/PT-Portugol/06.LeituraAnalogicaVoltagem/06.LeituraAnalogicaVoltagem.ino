/*
  Leitura Analógica de voltagem

  Este exemplo mostra como ler a voltagem de um pino analógico e enviar o mesmo pela
  serial. Você pode ler o resultado deste pelo Monitor Serial ou pelo Plotter Serial
  encontrado em (Ferramentas > Plotter Serial)

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

  // Inicializa a comunicação serial
  Serial.comecar(9600)

fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Le o valor do pino analogico, valor este entre 0 e 1023
  variavel(numerico, valorSensor)
  valorSensor = leiaAnalogico(A0)

  // Converte a leitura analogica para uma voltagem entre 0 e 5V
  variavel(decimal, voltagem)
  voltagem = valorSensor * (5.0 / 1023.0)

  // Imprime o estado do botão na Serial
  imprimeln(Serial, voltagem)

  // Espera por 1 milissegundo para aumentar a estabilidade
  espera(1)

fim
