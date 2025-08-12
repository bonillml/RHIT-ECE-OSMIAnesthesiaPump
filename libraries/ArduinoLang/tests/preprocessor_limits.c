#include <stdio.h>
#include <stdlib.h>

// Change preprocessor directive names
// we cannot change preprocessor directives at 
// least from now

// Error 1
// #define #defina #define
// #define defina define
// #defina vaca 1

// Error 2
// #define #include inclua
// #define include inclua

// Define symbols as names
#define inicio {
#define fim ;}

// Recreate lasso to apear more like portugol
#define se ;if( 
#define entao ){
#define senaose ;} else if (
#define senao ;} else {
#define fimse ;}

// Redefine declarations
#define algoritmo int main()
#define fim_algoritmo }

// Redefine functions
#define variavel_numerica "%d"
#define variavel_decimal "%f"
#define escreva(text, ...) ;printf(text "\n", ## __VA_ARGS__);

// Redefine functions more difficults
#define leia_inteiro(number) ;scanf("%d", &number);

// Redefine reserved words
#define retorna ;return

// Redefine constants
#define fim_sucesso EXIT_SUCCESS;

// Redefine symbols with symbols
// cannot redefine symbols with symbols
//#define <- =

// Redefine extreme complex variable declaration
// Accept any number of args >= N, but expand to just the Nth one.
// Here, N == 6.
#define _GET_NTH_ARG(_1, _2, _3, _4, _5, N, ...) N

// Define some macros to help us create overrides based on the
// arity of a for-each-style macro.
#define _fe_0(_call, ...)
#define _fe_1(_call, x) _call(x)
#define _fe_2(_call, x, ...) _call(x) _fe_1(_call, __VA_ARGS__)
#define _fe_3(_call, x, ...) _call(x) _fe_2(_call, __VA_ARGS__)
#define _fe_4(_call, x, ...) _call(x) _fe_3(_call, __VA_ARGS__)

/**
 * Provide a for-each construct for variadic macros. Supports up
 * to 4 args.
 *
 * Example usage1:
 *     #define FWD_DECLARE_CLASS(cls) class cls;
 *     CALL_MACRO_X_FOR_EACH(FWD_DECLARE_CLASS, Foo, Bar)
 *
 * Example usage 2:
 *     #define START_NS(ns) namespace ns {
 *     #define END_NS(ns) }
 *     #define MY_NAMESPACES System, Net, Http
 *     CALL_MACRO_X_FOR_EACH(START_NS, MY_NAMESPACES)
 *     typedef foo int;
 *     CALL_MACRO_X_FOR_EACH(END_NS, MY_NAMESPACES)
 */
#define CALL_MACRO_X_FOR_EACH(x, ...) \
    _GET_NTH_ARG("ignored", ##__VA_ARGS__, \
	_fe_4, _fe_3, _fe_2, _fe_1, _fe_0)(x, ##__VA_ARGS__)

// Redefine types
//#define numerico int
//#define decimal float
//#define texto string
#define numerico(name) int name;
#define decimal(name) float name;

// And then the function to declare any number of vars
#define declare(tipo, ...) ;CALL_MACRO_X_FOR_EACH(tipo, __VA_ARGS__)

// For in portugol style
#define para ;for(
#define de = 
#define ate(X, Y) ; X <= Y ; X++

// While in portugol style
#define enquanto ;while(
#define faca )

// Vector Initialization

// Check if __VA_ARGS__ is non empty
// <3 https://stackoverflow.com/questions/21474061/detect-presence-or-absence-of-arguments-in-a-c-macro
#define GET(_0, _1) _0  // Return the first of two arguments
#define GET_(_0, _1) _1  // Return the second of two arguments

#define JOIN(_0, _1) _0 ## _1  // Concatenate two arguments
#define EJOIN(_0, _1) JOIN(_0, _1)  // Expand macros and concatenate

#define FIRST(_, ...) _  // Truncate everything after first comma
#define EFIRST(_) FIRST(_)  // Expand argument and pass to FIRST

#define REST(_0, ...) __VA_ARGS__  // Remove everything before first comma

#define GET_GET(...) \
    EJOIN(GET, EFIRST(REST(,,##__VA_ARGS__ _)))  // Branch between GET and GET_

#define IFARGS(YES, NO, ...) GET_GET(__VA_ARGS__)(YES, NO)

#define PREPEND_EQUAL(...) = __VA_ARGS__
#define NO_EQUAL()
#define PREPEND_EQUAL_IF_NONEMPTY(...) IFARGS(PREPEND_EQUAL, NO_EQUAL, __VA_ARGS__)(__VA_ARGS__)

#define vetor(_TYPE, _NAME, _SIZE, ...) ; _TYPE _NAME[_SIZE + 1] PREPEND_EQUAL_IF_NONEMPTY(__VA_ARGS__);

algoritmo
inicio
	declare(numerico, x)

	escreva("Insira um numero")
	leia_inteiro(x)

	se x % 2 == 0 entao
		escreva("Numero par")
	senao
		escreva("Numero impar")
	fimse

	se x == 0 entao
		escreva("Escolheu 0")
	senaose x <= 10 entao
		escreva("Escolheu entre 1 e 10")
	senao
		escreva("Maior que 10")
	fimse

	escreva("Enquanto de 1 ate 10")
	declare(numerico, i)
	i <- 0
	enquanto i < 10 faca
	inicio
		escreva("Numero " variavel_numerica, i)
		i = i + 1
	fim

	escreva("Laco de 1 até 5")
	declare(numerico, j)
	para j de 1 ate(j, 5) faca
	inicio
		escreva("j == > " variavel_numerica, j)
	fim

	vetor(int, meu_vetor, 8, {10, 20})

	vetor(int, vetor_problematico, 8)	

	retorna fim_sucesso
fim_algoritmo