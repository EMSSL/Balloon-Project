Readme.txt
Christopher D. Yoder
cdyoder@ncsu.edu

% -----------------------------
What this repo directory is:
This repo directory is code that allows you to input a sail design, run AVL, and return the stability derivatives for the design. More information about AVL can be found here: https://web.mit.edu/drela/Public/web/avl/

NOTE: 
There are A LOT of random broken files here. These files come with no guarantee of operation, accuracy, or anything else you want from me. You're on your own. Goodluck!

% -----------------------------
How to use this repo directory
1. Open RunDesign_subscale.m
2. Edit the following fields to your liking:
	a. runs
		This structure defines the run parameters (name, sail base units, velocity, etc).
	b. wing
		This structure defines the wing. 
	c. rudder
		This structure defines the rudder. 
	d. const
		This structure defines what is constant during the sail analysis (alfa, beta, p, q, r, etc)
3. To run the single case, change Line 223 and set alpha to the desired sail angle of attack. To run multiple alphas, change Line 223 to a vector. 
4. The function alphaRun will write your input files to AVL, run AVL, and read out your stability derivatives to a file. CL, CD, xnp, and yL are the lift, drag coefficients as a function of span as well as the neutral point and lift position.

% -----------------------------
Specific MATLAB functions which are of note:
1. alphaRun(design, alpha)
	This function runs the AVL analysis for a given sail design (design structure) and alpha condition. 
2. ReadStrips(filename, 1, design)
	This function reads the filename.strip file and returns the aerodyanamic variables as a function of span position. 
3. ReadStabDerivs_revA(filename)
	This function reads the filename.stabs file and returns the stability derivatives as a structure for use.

% -----------------------------
AVL input/output files
The code will generate several input files for AVL. They will be named usin the basic file name filename (as specified by the user in the code). They are as follows:
1. filename.run
	This is the run file for AVL. It is a list of commands that AVL executes during operation. 
2. filename.mass
	This is the mass file for AVL. This file tells AVL what the components of the aircraft are, what their inertia tensors are, where their CM positions relative to the aircraft frame, etc. 
3. filename.avl
	This is the avl file. This file defines the geometry, airfoil, and placements in a way AVL can understand. 

AVL generates the following output files:
1. filename.strip
	This file contains aerodynamic variables of interest as functions along the span of the wing and rudder. This file is used to generate the lift distribution along the wing span. 
2. filename. stabs
	This file reports the aerodyanmic coefficients in the sail stability frame as a function of the design run (alpha, beta, etc.). The values in the first table (under Alpha, Beta, etc) are the total coefficients (C_L, C_D, etc). These are for the sail at this condition. The second table are the coefficients used in the Taylor-series expansion form (C_L_alpha, C_D_beta, etc). These coefficients are what are multiplied with the states (alpha, beta, etc) to generate the forces and moments on the sail. 