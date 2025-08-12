#ifndef PT_PORTUGOL_H
#define PT_PORTUGOL_H

#include "utility.h"

/*
 *
 * Created By Jeferson Lima
 *
 * Portuguese language with portugol (pseudolanguage-style)
 * this is a pseudo language bind to arduino program
 *
 */

// Prevent errors of expected comma
#define PREVENT_SEMICOLON_ERROR ;

// Define symbols as names
#define inicio {
#define fim PREVENT_SEMICOLON_ERROR }
#define fim_algoritmo PREVENT_SEMICOLON_ERROR }

// Recreate lasso to apear more like portugol
#define se PREVENT_SEMICOLON_ERROR if(
#define entao ){
#define senaose PREVENT_SEMICOLON_ERROR } else if (
#define senao PREVENT_SEMICOLON_ERROR } else {
#define fimse PREVENT_SEMICOLON_ERROR }

// Redefine functions
#define variavel_numerica "%d"
#define variavel_decimal "%f"
#define escreva(_TEXT, ...) PREVENT_SEMICOLON_ERROR printf(_TEXT "\n", ## __VA_ARGS__);

// Redefine functions more difficults
#define leia_inteiro(_NUMBER) PREVENT_SEMICOLON_ERROR scanf("%d", &_NUMBER);

// Redefine reserved words
#define retorna PREVENT_SEMICOLON_ERROR return

// Just types
#define numerico0() int
#define decimal0() float
#define caractere0() char
#define logico0() bool

// Redefine types
#define declareType(_TYPE, _NAME) PREVENT_SEMICOLON_ERROR _TYPE _NAME;
#define numerico1(_NAME) declareType(int, _NAME)
#define decimal1(_NAME) declareType(float, _NAME)
#define caractere1(_NAME) declareType(char, _NAME)
#define logico1(_NAME) declareType(bool, _NAME)

// Types with values
#define declareTypeValue(_TYPE, _NAME, _VALUE) PREVENT_SEMICOLON_ERROR _TYPE _NAME = _VALUE;
#define numerico2(_NAME, _VALUE) declareTypeValue(int, _NAME, _VALUE)
#define decimal2(_NAME, _VALUE) declareTypeValue(float, _NAME, _VALUE)
#define caractere2(_NAME, _VALUE) declareTypeValue(char, _NAME, _VALUE)
#define logico2(_NAME, _VALUE) declareTypeValue(bool, _NAME, _VALUE)

// Overloaded declare functions
#define numerico(...) GET_MACRO(_, ##__VA_ARGS__, _, _, numerico2, numerico1, numerico0)(__VA_ARGS__)
#define decimal(...) GET_MACRO(_, ##__VA_ARGS__, _, _, decimal2, decimal1, decimal0)(__VA_ARGS__)
#define caractere(...) GET_MACRO(_, ##__VA_ARGS__, _, _, caractere2, caractere1, caractere0)(__VA_ARGS__)
#define logico(...) GET_MACRO(_, ##__VA_ARGS__, _, _, logico2, logico1, logico0)(__VA_ARGS__)

// Other names for declare functions
#define numerica(...) numerico(##__VA_ARGS__)
#define logica(...) logica(##__VA_ARGS__)

// Vector Initialization
#define vetor(_TYPE, _NAME, _SIZE, ...) PREVENT_SEMICOLON_ERROR _TYPE() _NAME[_SIZE + 1] PREPEND_EQUAL_IF_NONEMPTY(__VA_ARGS__);
//#define vetor(...) GET_MACRO(_, ##__VA_ARGS__, vetor4, vetor3, _, _, _)(__VA_ARGS__)

// Constantes
#define constante(_TYPE, _NAME, _VALUE) PREVENT_SEMICOLON_ERROR _TYPE(const _NAME, _VALUE);

// And then the function to declare any number of vars
#define declare(_TYPE, ...) PREVENT_SEMICOLON_ERROR CALL_MACRO_X_FOR_EACH(_TYPE, __VA_ARGS__)
#define variavel(_TYPE, ...) declare(_TYPE, __VA_ARGS__)

// While in portugol style
#define enquanto PREVENT_SEMICOLON_ERROR while(
#define faca )

// For in portugol style
#define para PREVENT_SEMICOLON_ERROR for(
#define de =

// Ate com e sem incremento crescente
#define ateCrescente3(_X, _Y, _Z) ; _X <= _Y; _X += _Z
#define ateCrescente2(_X, _Y) ateCrescente3(_X, _Y, 1)
#define ateCrescente(...) GET_MACRO(_, ##__VA_ARGS__, _, ateCrescente3, ateCrescente2, _, _)(__VA_ARGS__)
#define ate(...) ateCrescente(__VA_ARGS__)

// Ate com e sem incremento decrescente
#define ateDecrescente3(_X, _Y, _Z) ; _X >= _Y; _X -= _Z
#define ateDecrescente2(_X, _Y) ateDecrescente3(_X, _Y, 1)
#define ateDecrescente(...) GET_MACRO(_, ##__VA_ARGS__, _, ateDecrescente3, ateDecrescente2, _, _)(__VA_ARGS__)

// Basic arduino reserved function name
#define configura void setup()
#define algoritmo void loop()

// Basic void Arduino Functions
#define modoPino(_PIN, _MODE) PREVENT_SEMICOLON_ERROR pinMode(_PIN, _MODE);
#define escrevaDigital(_PIN, _STATE) PREVENT_SEMICOLON_ERROR digitalWrite(_PIN, _STATE);
#define escrevaAnalogico(_PIN, _VALUE) PREVENT_SEMICOLON_ERROR analogWrite(_PIN, _VALUE);
#define espera(_TIME) PREVENT_SEMICOLON_ERROR delay(_TIME);
#define tom(_PIN, _NOTE, _DURATION) PREVENT_SEMICOLON_ERROR tone(_PIN, _NOTE, _DURATION);
#define semTom(_PIN) PREVENT_SEMICOLON_ERROR noTone(_PIN);
#define limites(_VAR, _MIN, _MAX) PREVENT_SEMICOLON_ERROR constrain(_VAR, _MIN, _MAX);
#define mapeie(_VAR, _MIN1, _MAX1, _MIN2, _MAX2) PREVENT_SEMICOLON_ERROR mapeie(_VAR, _MIN1, _MAX1, _MIN2, _MAX2)

// Basic functions with return
#define leiaAnalogico(_PIN) analogRead(_PIN);
#define leiaDigital(_PIN) digitalRead(_PIN);

// Methods as functions
#define comecar begin
#define comeca(_OBJECT, _SPEED) PREVENT_SEMICOLON_ERROR _OBJECT.begin(_SPEED);

#define imprimir print
#define imprime(_OBJECT, _TEXT) PREVENT_SEMICOLON_ERROR _OBJECT.print(_TEXT);

#define imprimirln println
#define imprimeln(_OBJECT, _TEXT) PREVENT_SEMICOLON_ERROR _OBJECT.println(_TEXT);

#define disponivel available()
#define estaDisponivel(_OBJECT) _OBJECT.available();

#define ler read
#define le(_OBJECT) _OBJECT.read();
#define leCaractere le(Serial)

#define escrever write
#define escreve(_OBJECT, _CHAR) PREVENT_SEMICOLON_ERROR _OBJECT.write(_CHAR);

// Avoid errors on objects
//#define Serial PREVENT_SEMICOLON_ERROR Serial

// Constants
#define ALTO HIGH
#define BAIXO LOW
#define LIGADO HIGH
#define DESLIGADO LOW
#define SAIDA OUTPUT
#define ENTRADA INPUT
#define ENTRADA_PULLUP INPUT_PULLUP
#define LED_INTEGRADO LED_BUILTIN

// Boolean functions
#define ou ||
#define e &&

// Time constants
#define MEIO_SEGUNDO 500
#define UM_SEGUNDO 1000
#define DOIS_SEGUNDOS 2000
#define TRES_SEGUNDOS 3000

#endif
