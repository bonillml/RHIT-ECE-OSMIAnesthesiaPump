/*
  Melodia

  Executa uma melodia.

  Circuito
    - resistor de 50 ohms e alto-falante piezoelétrico

  O objetivo deste código é o de apresentar a ideia de usar o arduino com linguagem
  portugol, adaptada através do uso de diversas macros. Mais informações consultar
  documentação do projeto ArduinoLang.

  criado 28 de Fevereiro de 2018
  por Jeferson Lima

  Este exemplo está em dominio público.
  
*/

// Inclui biblioteca para linguagem português com estilo da linguagem portugol
#include <ArduinoLangPT_Portugol.h>

// Inclui as notas
#include "notas.h"

// Declara o vetor de duracao das notas
vetor(numerico, duracaoNotas, 8, {
  4, 8, 8, 4, 4, 4, 4, 4 
})

// Declara o vetor da melodia
vetor(numerico, melodia, 8, {
  NOTA_C4, NOTA_G3, NOTA_G3, NOTA_A3, NOTA_G3, 0, NOTA_B3, NOTA_C4
})

// Especifica o pino do alto falante
variavel(numerico, pinoAltoFalante = 8)

// Configura o arduino
configura
inicio

  // Percore as notas da melodia
  variavel(numerico, estaNota)
  para estaNota de 1 ate(estaNota, 8) faca
  inicio

    // Para calcular a duração da nota, pegue um segundo e divida pelo tipo da nota.
    // Exemplo: quarta nota = 1000 / 4, oitava nota = 1000 / 8, etc. 
    variavel(numerico, duracaoNota)
    duracaoNota = 1000 / duracaoNotas[estaNota]
    
    // Executa o tom pelo tempo definido
    tom(pinoAltoFalante, melodia[estaNota], duracaoNota)

    // Como o tom é executado em paralelo, para sentirmos a diferença entre as notas
    // iremos esperar por 30% do tempo a mais, porque funciona bem
    variavel(numerico, pausaEntreNotas)
    pausaEntreNotas = duracaoNota * 1.30
    espera(pausaEntreNotas)

    // Para de executar o som
    semTom(pinoAltoFalante)
    
  fim

fim

// Executa o algoritmo em loop, indefinidamente
algoritmo
inicio

  // Sem necessidade para repetir a música

fim
