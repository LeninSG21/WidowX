interface AL_MD {
	title: string;
	intro: string;
	s1: string;
	functions: {
		name: string;
		description: string;
	}[];
}

export const spanishAL: AL_MD = {
	title: "Librería de Arduino",
	intro: `WidowX.h es una librería de Arduino diseñada para simplificar la operación del [brazo robótico WidowX] (https://www.trossenrobotics.com/widowxrobotarm)
    de [Trossen Robotics](https://www.trossenrobotics.com/), cuando se controla con el [Arbotix-M Robocontroller](https://learn.trossenrobotics.com/arbotix). Se 
    basa en la librería [ax12.h](https://github.com/vanadiumlabs/arbotix/blob/master/libraries/Bioloid/ax12.h). Adentro de algunas de sus funciones hace uso de las
    matrices definidas en la librería [BasicLinearAlgebra.h](https://github.com/tomstewart89/BasicLinearAlgebra). Por lo tanto, para funcionar correctamente,
    se deben instalar ambas dependencias.
    `,
	s1: "Funciones",
	functions: [
		{
			name: "WidowX()",
			description:
				"Constructor de la clase. Fija el número de servomotores a 6 y llena el arreglo de id del 1 al 6",
		},
		{
			name: "init(uint8_t relax)",
			description:
				"Esta NO es una función necesaria para que el brazo funcione adecuadamente. Primero, checa el voltaje con la función checkVoltage(). Posteriormente, manda el brazo a la posición de descanso (rest). Si el parámetro relax es 1, entonces desactiva el torque del brazo.",
		},
		{ name: "WidowX()", description: "Constructor" },
	],
};
export const englishAL: AL_MD = {
	title: "Arduino Library..",
	intro: `WidowX.h is an Arduino library designed to simplify the operation of the
     [WidowX Robot Arm](https://www.trossenrobotics.com/widowxrobotarm) of [Trossen Robotics](https://www.trossenrobotics.com/)
      when controlled with the [Arbotix-M Robocontroller](https://learn.trossenrobotics.com/arbotix). It is based in [ax12.h](https://github.com/vanadiumlabs/arbotix/blob/master/libraries/Bioloid/ax12.h). 
      Inside some of its functions it uses the Matrix object as defined in [BasicLinearAlgebra.h](https://github.com/tomstewart89/BasicLinearAlgebra). 
      Hence, it requires installation in order to work properly.`,
	s1: "Functions",
	functions: [
		{
			name: "WidowX()",
			description:
				"Class constructor. Sets the number of servos to 6 and fills the id array from 1 to 6",
		},
		{
			name: "init(uint8_t relax)",
			description:
				"This is NOT a required function for the arm to work properly. First, it checks the voltage with the function checkVoltage(). Then, it sends the arm to rest position. If relax is 1, then it disables the torque of the arm.",
		},
		{ name: "WidowX()", description: "Constructor" },
	],
};
