# MCMC_Data_Association
This is clearly a work in progress. This is my attempted implementation of the Online MCMCDA algorithm described in the pdf.

Room for improvement: When calculating the posterior it'd be faster to take
the log and use summations instead of multiplication.

To do:  Likelihood function (in progress)
	propasal moves
	Markov Chain convergenace/ mixing time
	copy constructor for graph class (So we can store the MAP estimation)
