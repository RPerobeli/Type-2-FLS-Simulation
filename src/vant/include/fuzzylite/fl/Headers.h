/*
 fuzzylite (R), a fuzzy logic control library in C++.
 Copyright (C) 2010-2017 FuzzyLite Limited. All rights reserved.
 Author: Juan Rada-Vilela, Ph.D. <jcrada@fuzzylite.com>

 This file is part of fuzzylite.

 fuzzylite is free software: you can redistribute it and/or modify it under
 the terms of the FuzzyLite License included with the software.

 You should have received a copy of the FuzzyLite License along with
 fuzzylite. If not, see <http://www.fuzzylite.com/license/>.

 fuzzylite is a registered trademark of FuzzyLite Limited.
 */

#ifndef FL_HEADERS_H
#define FL_HEADERS_H

/**
    The Headers.h file contains the headers of all the classes in the
    `fuzzylite` library, thereby encouraging the use of the directive `#include
    "fl/Headers.h"` in projects using the library.
 */


#include "fuzzylite/fl/fuzzylite.h"

#include "fuzzylite/fl/Benchmark.h"
#include "fuzzylite/fl/Complexity.h"
#include "fuzzylite/fl/Console.h"
#include "fuzzylite/fl/Engine.h"
#include "fuzzylite/fl/Exception.h"

#include "fuzzylite/fl/activation/Activation.h"
#include "fuzzylite/fl/activation/First.h"
#include "fuzzylite/fl/activation/General.h"
#include "fuzzylite/fl/activation/Highest.h"
#include "fuzzylite/fl/activation/Last.h"
#include "fuzzylite/fl/activation/Lowest.h"
#include "fuzzylite/fl/activation/Proportional.h"
#include "fuzzylite/fl/activation/Threshold.h"

#include "fuzzylite/fl/defuzzifier/Bisector.h"
#include "fuzzylite/fl/defuzzifier/Centroid.h"
#include "fuzzylite/fl/defuzzifier/Defuzzifier.h"
#include "fuzzylite/fl/defuzzifier/IntegralDefuzzifier.h"
#include "fuzzylite/fl/defuzzifier/SmallestOfMaximum.h"
#include "fuzzylite/fl/defuzzifier/LargestOfMaximum.h"
#include "fuzzylite/fl/defuzzifier/MeanOfMaximum.h"
#include "fuzzylite/fl/defuzzifier/WeightedAverage.h"
#include "fuzzylite/fl/defuzzifier/WeightedDefuzzifier.h"
#include "fuzzylite/fl/defuzzifier/WeightedSum.h"

#include "fuzzylite/fl/factory/ActivationFactory.h"
#include "fuzzylite/fl/factory/CloningFactory.h"
#include "fuzzylite/fl/factory/ConstructionFactory.h"
#include "fuzzylite/fl/factory/FactoryManager.h"
#include "fuzzylite/fl/factory/FunctionFactory.h"
#include "fuzzylite/fl/factory/DefuzzifierFactory.h"
#include "fuzzylite/fl/factory/HedgeFactory.h"
#include "fuzzylite/fl/factory/SNormFactory.h"
#include "fuzzylite/fl/factory/TNormFactory.h"
#include "fuzzylite/fl/factory/TermFactory.h"

#include "fuzzylite/fl/imex/CppExporter.h"
#include "fuzzylite/fl/imex/FclImporter.h"
#include "fuzzylite/fl/imex/FclExporter.h"
#include "fuzzylite/fl/imex/FisImporter.h"
#include "fuzzylite/fl/imex/FisExporter.h"
#include "fuzzylite/fl/imex/FldExporter.h"
#include "fuzzylite/fl/imex/FllImporter.h"
#include "fuzzylite/fl/imex/FllExporter.h"
#include "fuzzylite/fl/imex/JavaExporter.h"
#include "fuzzylite/fl/imex/RScriptExporter.h"

#include "fuzzylite/fl/hedge/Any.h"
#include "fuzzylite/fl/hedge/Extremely.h"
#include "fuzzylite/fl/hedge/Hedge.h"
#include "fuzzylite/fl/hedge/HedgeFunction.h"
#include "fuzzylite/fl/hedge/Not.h"
#include "fuzzylite/fl/hedge/Seldom.h"
#include "fuzzylite/fl/hedge/Somewhat.h"
#include "fuzzylite/fl/hedge/Very.h"

#include "fuzzylite/fl/Operation.h"

#include "fuzzylite/fl/norm/Norm.h"
#include "fuzzylite/fl/norm/SNorm.h"
#include "fuzzylite/fl/norm/TNorm.h"

#include "fuzzylite/fl/norm/s/AlgebraicSum.h"
#include "fuzzylite/fl/norm/s/BoundedSum.h"
#include "fuzzylite/fl/norm/s/DrasticSum.h"
#include "fuzzylite/fl/norm/s/EinsteinSum.h"
#include "fuzzylite/fl/norm/s/HamacherSum.h"
#include "fuzzylite/fl/norm/s/Maximum.h"
#include "fuzzylite/fl/norm/s/NilpotentMaximum.h"
#include "fuzzylite/fl/norm/s/NormalizedSum.h"
#include "fuzzylite/fl/norm/s/SNormFunction.h"
#include "fuzzylite/fl/norm/s/UnboundedSum.h"

#include "fuzzylite/fl/norm/t/AlgebraicProduct.h"
#include "fuzzylite/fl/norm/t/BoundedDifference.h"
#include "fuzzylite/fl/norm/t/DrasticProduct.h"
#include "fuzzylite/fl/norm/t/EinsteinProduct.h"
#include "fuzzylite/fl/norm/t/HamacherProduct.h"
#include "fuzzylite/fl/norm/t/Minimum.h"
#include "fuzzylite/fl/norm/t/NilpotentMinimum.h"
#include "fuzzylite/fl/norm/t/TNormFunction.h"

#include "fuzzylite/fl/rule/Antecedent.h"
#include "fuzzylite/fl/rule/Consequent.h"
#include "fuzzylite/fl/rule/Rule.h"
#include "fuzzylite/fl/rule/RuleBlock.h"
#include "fuzzylite/fl/rule/Expression.h"

#include "fuzzylite/fl/term/Aggregated.h"
#include "fuzzylite/fl/term/Bell.h"
#include "fuzzylite/fl/term/Binary.h"
#include "fuzzylite/fl/term/Concave.h"
#include "fuzzylite/fl/term/Constant.h"
#include "fuzzylite/fl/term/Cosine.h"
#include "fuzzylite/fl/term/Discrete.h"
#include "fuzzylite/fl/term/Function.h"
#include "fuzzylite/fl/term/Gaussian.h"
#include "fuzzylite/fl/term/GaussianProduct.h"
#include "fuzzylite/fl/term/Linear.h"
#include "fuzzylite/fl/term/PiShape.h"
#include "fuzzylite/fl/term/Ramp.h"
#include "fuzzylite/fl/term/Rectangle.h"
#include "fuzzylite/fl/term/SShape.h"
#include "fuzzylite/fl/term/Sigmoid.h"
#include "fuzzylite/fl/term/SigmoidDifference.h"
#include "fuzzylite/fl/term/SigmoidProduct.h"
#include "fuzzylite/fl/term/Spike.h"
#include "fuzzylite/fl/term/Term.h"
#include "fuzzylite/fl/term/Activated.h"
#include "fuzzylite/fl/term/Trapezoid.h"
#include "fuzzylite/fl/term/Triangle.h"
#include "fuzzylite/fl/term/ZShape.h"

#include "fuzzylite/fl/variable/InputVariable.h"
#include "fuzzylite/fl/variable/OutputVariable.h"
#include "fuzzylite/fl/variable/Variable.h"


#endif /* FL_HEADERS_H */
