#pragma once

#include <string>

struct G1Options
{
  std::string packageName = "g1_description";
  std::string packageUri
      = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "master/robots/g1_description";
  std::string robotUri = "package://g1_description/g1_29dof.urdf";
};

void finalizeG1Options(
    G1Options& options,
    bool packageNameExplicit,
    bool packageUriExplicit,
    bool robotUriExplicit);

int runG1(const G1Options& options);
