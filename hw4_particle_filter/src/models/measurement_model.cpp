


double MCL::prob_hit(double z_true, double z)
{
	double p = 0.0;

	if (z >= 0 && z <= z_max)
	{
		double eta;


	}

	return p;
}

double MCL::prob_short(double z_true, double z)
{
	double p = 0.0;

	if (z >= 0 && z <= z_true)
	{
		double eta = 1 / (1 - exp(-lambda_short * z_true));
		p = (eta * lambda_short * exp(-lambda_short * z));
	}

	return p;
}

double MCL::prob_max(double z)
{
	if ( abs(z-z_max) < 0.001 )	
		return 1.0;
	else
		return 0.0;
}

double MCL::prob_rand(double z)
{
	if (z >= 0 && z <= z_max)
		return (1.0/z_max);
	else
		return 0.0;
}