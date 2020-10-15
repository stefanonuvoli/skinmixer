#include <gurobi_c++.h>

#include "quad_ilp.h"

#include "quad_utils.h"

#define MINSIDEVALUE 1
#define AVERAGELENGTHSMOOTHITERATIONS 1
#define MAXCOST 1000000.0

namespace QuadBoolean {
namespace internal {


std::vector<double> getSmoothedChartAverageEdgeLength(
        const ChartData& chartData);

inline std::vector<int> solveILP(
        ChartData& chartData,
        const double alpha,
        const ILPMethod& method,
        const bool isometry,
        const double timeLimit,
        double& gap,
        ILPStatus& status)
{
    using namespace std;

    vector<int> result(chartData.subSides.size(), -1);

    try {
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);

//        model.set(GRB_IntParam_OutputFlag, 0);

        model.set(GRB_DoubleParam_TimeLimit, timeLimit);

        // Create variables
        GRBQuadExpr obj = 0;

        vector<GRBVar> vars(chartData.subSides.size());
        vector<GRBVar> diff;
        vector<GRBVar> abs;

        vector<GRBVar> free(chartData.charts.size());

        std::vector<double> avgLength = getSmoothedChartAverageEdgeLength(chartData);

        for (size_t subsideId = 0; subsideId < chartData.subSides.size(); subsideId++) {
            const ChartSubSide& subside = chartData.subSides[subsideId];

            //If it is not a border (free)
            if (!subside.isFixed) {
                assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);

                vars[subsideId] = model.addVar(MINSIDEVALUE, GRB_INFINITY, 0.0, GRB_INTEGER, "s" + to_string(subsideId));
            }
            else {
                vars[subsideId] = model.addVar(std::max(subside.size - 1, MINSIDEVALUE), subside.size + 1, 0.0, GRB_INTEGER, "s" + to_string(subsideId));

                size_t dId = diff.size();
                size_t aId = abs.size();
                diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "do" + to_string(dId)));
                abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "ao" + to_string(aId)));
                model.addConstr(diff[dId] == vars[subsideId] - subside.size, "doc" + to_string(dId));
                model.addGenConstrAbs(abs[aId], diff[dId], "aoc" + to_string(aId));

                obj += abs[aId] * MAXCOST;
            }
        }

        std::cout << chartData.subSides.size() << " subsides!" << std::endl;

        if (isometry) {
            GRBQuadExpr isoExpr = 0;
            const double isoCost = alpha;
            //Regularity
            for (size_t subsideId = 0; subsideId < chartData.subSides.size(); subsideId++) {
                const ChartSubSide& subside = chartData.subSides[subsideId];

                //If it is not a border (free)
                if (!subside.isFixed) {
                    assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);
                    assert(avgLength[subside.incidentCharts[0]] > 0 && avgLength[subside.incidentCharts[1]] > 0);

                    double incidentChartAverageLength = (avgLength[subside.incidentCharts[0]] + avgLength[subside.incidentCharts[1]])/2;
                    double sideSubdivision = subside.length / incidentChartAverageLength;

                    GRBLinExpr value = vars[subsideId] - sideSubdivision;

                    if (method == LEASTSQUARES) {
                        isoExpr += value * value;
                    }
                    else {
                        size_t dId = diff.size();
                        size_t aId = abs.size();
                        diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d" + to_string(dId)));
                        abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "a" + to_string(aId)));
                        model.addConstr(diff[dId] == value, "dc" + to_string(dId));
                        model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                        isoExpr += abs[aId];
                    }
                }
            }
            obj += isoCost * isoExpr;
        }

        GRBQuadExpr regExpr = 0;
        const double regCost = 1 - alpha;
        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0) {
                size_t nSides = chart.chartSides.size();

                //Quad case
                if (nSides == 4) {
                    for (size_t j = 0; j < nSides; j++) {
                        const ChartSide& side1 = chart.chartSides[j];
                        const ChartSide& side2 = chart.chartSides[(j+2)%4];

                        GRBLinExpr subSide1Sum = 0;
                        for (const size_t& subSideId : side1.subsides) {
                            subSide1Sum += vars[subSideId];
                        }
                        GRBLinExpr subSide2Sum = 0;
                        for (const size_t& subSideId : side2.subsides) {
                            subSide2Sum += vars[subSideId];
                        }

                        GRBLinExpr value = subSide1Sum - subSide2Sum;

                        if (method == LEASTSQUARES) {
                            regExpr += (value * value) / nSides;
                        }
                        else {
                            size_t dId = diff.size();
                            size_t aId = abs.size();
                            diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                            abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));
                            model.addConstr(diff[dId] == value, "dc" + to_string(dId));
                            model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                            regExpr += abs[aId] / nSides;
                        }
                    }
                }
            }

            //Even side size sum constraint in a chart
            for (size_t i = 0; i < chartData.charts.size(); i++) {
                const Chart& chart = chartData.charts[i];
                if (chart.faces.size() > 0) {
                    if (chart.chartSides.size() < 3 || chart.chartSides.size() > 6) {
                        std::cout << "Chart " << i << " has " << chart.chartSides.size() << " sides." << std::endl;
                        continue;
                    }

                    GRBLinExpr sumExp = 0;
                    for (const size_t& subSideId : chart.chartSubSides) {
                        sumExp += vars[subSideId];
                    }

                    free[i] = model.addVar(2, GRB_INFINITY, 0.0, GRB_INTEGER, "f" + to_string(i));
                    model.addConstr(free[i] * 2 == sumExp);
                }
            }
        }

        obj += regCost * regExpr;


//        model.update();

        //Set objective function
        model.setObjective(obj, GRB_MINIMIZE);


//        model.write("out.lp");

        //Optimize model
        model.optimize();

        for (size_t i = 0; i < chartData.subSides.size(); i++) {
            result[i] = static_cast<int>(std::round(vars[i].get(GRB_DoubleAttr_X)));
        }

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        gap = model.get(GRB_DoubleAttr_MIPGap);

        status = ILPStatus::SOLUTIONFOUND;

        for (size_t i = 0; i < chartData.charts.size(); i++) {
            Chart& chart = chartData.charts[i];

            if (chart.faces.size() > 0) {
                int sizeSum = 0;
                for (const size_t& subSideId : chart.chartSubSides) {
                    sizeSum += result[subSideId];
                }

                if (sizeSum % 2 == 1) {
                    std::cout << "Error not even, chart: " << i << " -> ";
                    for (const size_t& subSideId : chart.chartSubSides) {
                        std::cout << result[subSideId] << " ";
                    }
                    std::cout << " = " << sizeSum << " - FREE: " << free[i].get(GRB_DoubleAttr_X) << std::endl;

                    status = ILPStatus::SOLUTIONWRONG;
                }
            }
        }

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;

        status = ILPStatus::INFEASIBLE;
    }

    return result;
}

std::vector<double> getSmoothedChartAverageEdgeLength(
        const ChartData& chartData)
{
    std::vector<double> avgLengths(chartData.charts.size() , -1);

    const size_t iterations = AVERAGELENGTHSMOOTHITERATIONS;

    //Fill charts with a border
    for (size_t i = 0; i < chartData.charts.size(); i++) {
        const Chart& chart = chartData.charts[i];
        if (chart.faces.size() > 0) {
            double currentQuadLength = 0;
            int numSides = 0;

            for (size_t sId : chart.chartSubSides) {
                const ChartSubSide& subside = chartData.subSides[sId];
                if (subside.isFixed) {
                    currentQuadLength += subside.length / subside.size;
                    numSides++;
                }
            }

            if (numSides > 0) {
                currentQuadLength /= numSides;
                avgLengths[i] = currentQuadLength;
            }
        }
    }

    //Fill charts with no borders
    bool done;
    do {
        done = true;
        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0 && avgLengths[i] < 0) {
                double currentLength = 0;
                size_t numAdjacentCharts = 0;

                for (size_t adjId : chart.adjacentCharts) {
                    if (avgLengths[adjId] > 0) {
                        currentLength += avgLengths[adjId];
                        numAdjacentCharts++;
                        done = false;
                    }
                }

                if (currentLength > 0) {
                    currentLength /= numAdjacentCharts;
                    avgLengths[i] = currentLength;
                }
            }
        }
    } while (!done);


    //Smoothing
    for (size_t k = 0; k < iterations; k++) {
        std::vector<double> lastAvgLengths = avgLengths;

        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0) {
                assert(lastAvgLengths[i] > 0);

                double currentLength = 0;
                size_t numAdjacentCharts = 0;

                for (size_t adjId : chart.adjacentCharts) {
                    if (avgLengths[adjId] > 0) {
                        currentLength += lastAvgLengths[adjId];
                        numAdjacentCharts++;
                    }
                }

                if (currentLength > 0) {
                    currentLength += lastAvgLengths[i];

                    currentLength /= (numAdjacentCharts + 1);
                    avgLengths[i] = currentLength;
                }
            }
        }
    }

    return avgLengths;
}

}
}


