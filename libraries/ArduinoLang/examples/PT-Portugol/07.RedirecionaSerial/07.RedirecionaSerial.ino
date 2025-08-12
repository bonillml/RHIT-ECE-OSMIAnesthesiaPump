/*
  Redireciona Serial

  Redireciona a mensagem enviada pela serial de volta para o computador.

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

  // Configura a comunicação serial
  comeca(Serial, 9600)
  
fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Se existir comunicação vindo da placa principal
  se Serial.disponivel entao

    // Le um caractere da comunicacao serial
    caractere(caractereLido)
    caractereLido = le(Serial)

    // Envia o caractere de volta
    escreve(Serial, caractereLido)
  
  fimse
  
fim
