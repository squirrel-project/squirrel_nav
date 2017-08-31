/**
 * @file cli_test.cpp
 * @author Matthew Amidon, Ryan Curtin
 *
 * Test for the CLI input parameter system.
 *
 * mlpack is free software; you may redistribute it and/or modify it under the
 * terms of the 3-clause BSD license.  You should have received a copy of the
 * 3-clause BSD license along with mlpack.  If not, see
 * http://www.opensource.org/licenses/BSD-3-Clause for more information.
 */
#include <mlpack/core.hpp>

#include <boost/test/unit_test.hpp>
#include "test_tools.hpp"

using namespace mlpack;
using namespace mlpack::util;
using namespace mlpack::kernel;
using namespace mlpack::data;
using namespace std;

// When we run these tests, we have to nuke the existing CLI object that's
// created by default.
struct CLITestDestroyer
{
  CLITestDestroyer() { CLI::Destroy(); }
};

BOOST_FIXTURE_TEST_SUITE(CLITest, CLITestDestroyer);

/**
 * Before running a test that uses the CLI options, we have to add the default
 * options that are required for CLI to function, since it will be destroyed at
 * the end of every test that uses CLI in this test suite.
 */
void AddRequiredCLIOptions()
{
  CLI::Add<bool>(false, "help", "Default help info.", 'h');
  CLI::Add<string>("", "info", "Get help on a specific module or option.");
  CLI::Add<bool>(false, "verbose", "Display informational messages and the full"
      " list of parameters and timers at the end of execution.", 'v');
  CLI::Add<bool>(false, "version", "Display the version of mlpack.", 'V');
}

/**
 * Tests that CLI works as intended, namely that CLI::Add propagates
 * successfully.
 */
BOOST_AUTO_TEST_CASE(TestCLIAdd)
{
  AddRequiredCLIOptions();

  // Check that the CLI::HasParam returns false if no value has been specified
  // on the commandline and ignores any programmatical assignments.
  CLI::Add<bool>(false, "global/bool", "True or False", 'a');

  // CLI::HasParam should return false here.
  BOOST_REQUIRE(!CLI::HasParam("global/bool"));

  // Check that our aliasing works.
  BOOST_REQUIRE_EQUAL(CLI::HasParam("global/bool"),
      CLI::HasParam("a"));
  BOOST_REQUIRE_EQUAL(CLI::GetParam<bool>("global/bool"),
      CLI::GetParam<bool>("a"));
}

/**
 * Tests that the various PARAM_* macros work properly.
 */
BOOST_AUTO_TEST_CASE(TestOption)
{
  AddRequiredCLIOptions();

  // This test will involve creating an option, and making sure CLI reflects
  // this.
  PARAM_IN(int, "test_parent/test", "test desc", "", 42, false);

  BOOST_REQUIRE_EQUAL(CLI::GetParam<int>("test_parent/test"), 42);
}

/**
 * Test that duplicate flags are filtered out correctly.
 */
BOOST_AUTO_TEST_CASE(TestDuplicateFlag)
{
  AddRequiredCLIOptions();

  PARAM_FLAG("test", "test", "t");

  int argc = 3;
  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "--test";
  argv[2] = "--test";

  // This should not throw an exception.
  CLI::ParseCommandLine(argc, const_cast<char**>(argv));
}

/**
 * Test that duplicate options throw an exception.
 */
BOOST_AUTO_TEST_CASE(TestDuplicateParam)
{
  AddRequiredCLIOptions();

  int argc = 5;
  const char* argv[5];
  argv[0] = "./test";
  argv[1] = "--info";
  argv[2] = "test1";
  argv[3] = "--info";
  argv[4] = "test2";

  // This should throw an exception.
  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::ParseCommandLine(argc, const_cast<char**>(argv)),
      runtime_error);
  Log::Fatal.ignoreInput = false;
}

/**
 * Ensure that a Boolean option which we define is set correctly.
 */
BOOST_AUTO_TEST_CASE(TestBooleanOption)
{
  AddRequiredCLIOptions();

  PARAM_FLAG("flag_test", "flag test description", "");

  BOOST_REQUIRE_EQUAL(CLI::HasParam("flag_test"), false);

  // Now check that CLI reflects that it is false by default.
  BOOST_REQUIRE_EQUAL(CLI::GetParam<bool>("flag_test"), false);

  // Now, if we specify this flag, it should be true.
  int argc = 2;
  char* argv[2];
  argv[0] = strcpy(new char[strlen("programname") + 1], "programname");
  argv[1] = strcpy(new char[strlen("--flag_test") + 1], "--flag_test");

  CLI::ParseCommandLine(argc, argv);

  BOOST_REQUIRE_EQUAL(CLI::GetParam<bool>("flag_test"), true);
  BOOST_REQUIRE_EQUAL(CLI::HasParam("flag_test"), true);

  delete[] argv[0];
  delete[] argv[1];
}

/**
 * Test that a vector option works correctly.
 */
BOOST_AUTO_TEST_CASE(TestVectorOption)
{
  AddRequiredCLIOptions();

  PARAM_VECTOR_IN(size_t, "test_vec", "test description", "t");

  int argc = 5;
  const char* argv[5];
  argv[0] = "./test";
  argv[1] = "--test_vec";
  argv[2] = "1";
  argv[3] = "2";
  argv[4] = "4";

  Log::Fatal.ignoreInput = true;
  CLI::ParseCommandLine(argc, const_cast<char**>(argv));
  Log::Fatal.ignoreInput = false;

  BOOST_REQUIRE(CLI::HasParam("test_vec"));

  vector<size_t> v = CLI::GetParam<vector<size_t>>("test_vec");

  BOOST_REQUIRE_EQUAL(v.size(), 3);
  BOOST_REQUIRE_EQUAL(v[0], 1);
  BOOST_REQUIRE_EQUAL(v[1], 2);
  BOOST_REQUIRE_EQUAL(v[2], 4);
}

/**
 * Test that we can use a vector option by specifying it many times.
 */
BOOST_AUTO_TEST_CASE(TestVectorOption2)
{
  AddRequiredCLIOptions();

  PARAM_VECTOR_IN(size_t, "test2_vec", "test description", "T");

  int argc = 7;
  const char* argv[7];
  argv[0] = "./test";
  argv[1] = "--test2_vec";
  argv[2] = "1";
  argv[3] = "--test2_vec";
  argv[4] = "2";
  argv[5] = "--test2_vec";
  argv[6] = "4";

//  Log::Fatal.ignoreInput = true;
  CLI::ParseCommandLine(argc, const_cast<char**>(argv));
//  Log::Fatal.ignoreInput = false;

  BOOST_REQUIRE(CLI::HasParam("test2_vec"));

  vector<size_t> v = CLI::GetParam<vector<size_t>>("test2_vec");

  BOOST_REQUIRE_EQUAL(v.size(), 3);
  BOOST_REQUIRE_EQUAL(v[0], 1);
  BOOST_REQUIRE_EQUAL(v[1], 2);
  BOOST_REQUIRE_EQUAL(v[2], 4);
}

BOOST_AUTO_TEST_CASE(InputMatrixParamTest)
{
  AddRequiredCLIOptions();

  // --matrix is an input parameter; it won't be transposed.
  CLI::Add<arma::mat>(arma::mat(), "matrix", "Test matrix", 'm', false, true,
      false);

  // Set some fake arguments.
  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "-m";
  argv[2] = "test_data_3_1000.csv";

  int argc = 3;

  // The const-cast is a little hacky but should be fine...
  Log::Fatal.ignoreInput = true;
  CLI::ParseCommandLine(argc, const_cast<char**>(argv));
  Log::Fatal.ignoreInput = false;

  // The --matrix parameter should exist.
  BOOST_REQUIRE(CLI::HasParam("matrix"));
  // The --matrix_file parameter should not exist (it should be transparent from
  // inside the program).
  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::HasParam("matrix_file"), runtime_error);
  Log::Fatal.ignoreInput = false;

  arma::mat dataset = CLI::GetParam<arma::mat>("matrix");
  arma::mat dataset2 = CLI::GetParam<arma::mat>("matrix");

  BOOST_REQUIRE_EQUAL(dataset.n_rows, 3);
  BOOST_REQUIRE_EQUAL(dataset.n_cols, 1000);
  BOOST_REQUIRE_EQUAL(dataset2.n_rows, 3);
  BOOST_REQUIRE_EQUAL(dataset2.n_cols, 1000);

  for (size_t i = 0; i < dataset.n_elem; ++i)
    BOOST_REQUIRE_CLOSE(dataset[i], dataset2[i], 1e-10);
}

BOOST_AUTO_TEST_CASE(InputMatrixNoTransposeParamTest)
{
  AddRequiredCLIOptions();

  // --matrix is a non-transposed input parameter.
  CLI::Add<arma::mat>(arma::mat(), "matrix", "Test matrix", 'm', false, true,
      true);

  // Set some fake arguments.
  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "--matrix_file";
  argv[2] = "test_data_3_1000.csv";

  int argc = 3;

  // The const-cast is a little hacky but should be fine...
  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // The --matrix parameter should exist.
  BOOST_REQUIRE(CLI::HasParam("matrix"));
  // The --matrix_file parameter should not exist (it should be transparent from
  // inside the program).
  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::HasParam("matrix_file"), runtime_error);
  Log::Fatal.ignoreInput = false;

  arma::mat dataset = CLI::GetParam<arma::mat>("matrix");
  arma::mat dataset2 = CLI::GetParam<arma::mat>("matrix");

  BOOST_REQUIRE_EQUAL(dataset.n_rows, 1000);
  BOOST_REQUIRE_EQUAL(dataset.n_cols, 3);
  BOOST_REQUIRE_EQUAL(dataset2.n_rows, 1000);
  BOOST_REQUIRE_EQUAL(dataset2.n_cols, 3);

  for (size_t i = 0; i < dataset.n_elem; ++i)
    BOOST_REQUIRE_CLOSE(dataset[i], dataset2[i], 1e-10);
}

BOOST_AUTO_TEST_CASE(OutputMatrixParamTest)
{
  AddRequiredCLIOptions();

  // --matrix is an output parameter.
  CLI::Add<arma::mat>(arma::mat(), "matrix", "Test matrix", 'm', false, false,
      false);

  // Set some fake arguments.
  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "-m";
  argv[2] = "test.csv";

  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // The --matrix parameter should exist.
  BOOST_REQUIRE(CLI::HasParam("matrix"));
  // The --matrix_file parameter should not exist (it should be transparent from
  // inside the program).
  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::HasParam("matrix_file"), runtime_error);
  Log::Fatal.ignoreInput = false;

  // Since it's an output parameter, we don't need any input and don't need to
  // call ParseCommandLine().
  arma::mat dataset = arma::randu<arma::mat>(3, 100);
  CLI::GetParam<arma::mat>("matrix") = dataset;

  // Write the file.
  CLI::Destroy();
  AddRequiredCLIOptions();

  // Now load the matrix back and make sure it was saved correctly.
  arma::mat dataset2;
  data::Load("test.csv", dataset2);

  BOOST_REQUIRE_EQUAL(dataset.n_cols, dataset2.n_cols);
  BOOST_REQUIRE_EQUAL(dataset.n_rows, dataset2.n_rows);
  for (size_t i = 0; i < dataset.n_elem; ++i)
    BOOST_REQUIRE_CLOSE(dataset[i], dataset2[i], 1e-10);

  // Remove the file.
  remove("test.csv");
}

BOOST_AUTO_TEST_CASE(OutputMatrixNoTransposeParamTest)
{
  AddRequiredCLIOptions();

  // --matrix is an output parameter.
  CLI::Add<arma::mat>(arma::mat(), "matrix", "Test matrix", 'm', false, false,
      true);

  // Set some fake arguments.
  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "-m";
  argv[2] = "test.csv";

  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // The --matrix parameter should exist.
  BOOST_REQUIRE(CLI::HasParam("matrix"));
  // The --matrix_file parameter should not exist (it should be transparent from
  // inside the program).
  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::HasParam("matrix_file"), runtime_error);
  Log::Fatal.ignoreInput = false;

  // Since it's an output parameter, we don't need any input and don't need to
  // call ParseCommandLine().
  arma::mat dataset = arma::randu<arma::mat>(3, 100);
  CLI::GetParam<arma::mat>("matrix") = dataset;

  // Write the file.
  CLI::Destroy();
  AddRequiredCLIOptions();

  // Now load the matrix back and make sure it was saved correctly.
  arma::mat dataset2;
  data::Load("test.csv", dataset2, true, false);

  BOOST_REQUIRE_EQUAL(dataset.n_cols, dataset2.n_cols);
  BOOST_REQUIRE_EQUAL(dataset.n_rows, dataset2.n_rows);
  for (size_t i = 0; i < dataset.n_elem; ++i)
    BOOST_REQUIRE_CLOSE(dataset[i], dataset2[i], 1e-10);

  // Remove the file.
  remove("test.csv");
}

BOOST_AUTO_TEST_CASE(IntParamTest)
{
  AddRequiredCLIOptions();

  CLI::Add<int>(0, "int", "Test int", 'i', false, true, false);

  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "-i";
  argv[2] = "3";

  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  BOOST_REQUIRE(CLI::HasParam("int"));
  BOOST_REQUIRE_EQUAL(CLI::GetParam<int>("int"), 3);
}

BOOST_AUTO_TEST_CASE(StringParamTest)
{
  AddRequiredCLIOptions();

  CLI::Add<string>("", "string", "Test string", 's', false, true, false);

  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "--string";
  argv[2] = "3";

  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  BOOST_REQUIRE(CLI::HasParam("string"));
  BOOST_REQUIRE_EQUAL(CLI::GetParam<string>("string"), string("3"));
}

BOOST_AUTO_TEST_CASE(DoubleParamTest)
{
  AddRequiredCLIOptions();

  CLI::Add<double>(0.0, "double", "Test double", 'd', false, true, false);

  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "--double";
  argv[2] = "3.12";

  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  BOOST_REQUIRE(CLI::HasParam("double"));
  BOOST_REQUIRE_CLOSE(CLI::GetParam<double>("double"), 3.12, 1e-10);
}

BOOST_AUTO_TEST_CASE(RequiredOptionTest)
{
  AddRequiredCLIOptions();

  CLI::Add<double>(0.0, "double", "Required test double", 'd', true, true,
      false);

  const char* argv[1];
  argv[0] = "./test";

  int argc = 1;

  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::ParseCommandLine(argc, const_cast<char**>(argv)),
      runtime_error);
  Log::Fatal.ignoreInput = false;
}

BOOST_AUTO_TEST_CASE(UnknownOptionTest)
{
  AddRequiredCLIOptions();

  const char* argv[2];
  argv[0] = "./test";
  argv[1] = "--unknown";

  int argc = 2;

  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::ParseCommandLine(argc, const_cast<char**>(argv)),
      runtime_error);
  Log::Fatal.ignoreInput = false;
}

/**
 * Test that GetUnmappedParam() works.
 */
BOOST_AUTO_TEST_CASE(UnmappedParamTest)
{
  AddRequiredCLIOptions();

  CLI::Add<arma::mat>(arma::mat(), "matrix", "Test matrix", 'm', false, true,
      true);
  CLI::Add<arma::mat>(arma::mat(), "matrix2", "Test matrix", 'M', false, false,
      true);
  CLI::Add<double>(0.0, "double", "Test double", 'd', false, true, false);
  CLI::Add<double>(0.0, "double2", "Test double", 'D', false, true, false);
  CLI::Add<GaussianKernel>(GaussianKernel(), "kernel", "Test kernel", 'k',
      false, true, true);
  CLI::Add<GaussianKernel>(GaussianKernel(), "kernel2", "Test kernel", 'K',
      false, false, true);

  const char* argv[11];
  argv[0] = "./test";
  argv[1] = "--matrix_file";
  argv[2] = "file1.csv";
  argv[3] = "-M";
  argv[4] = "file2.csv";
  argv[5] = "-d";
  argv[6] = "1.334";
  argv[7] = "-k";
  argv[8] = "kernel.txt";
  argv[9] = "-K";
  argv[10] = "kernel2.txt";

  int argc = 11;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // Now check that we can get unmapped parameters.
  BOOST_REQUIRE_EQUAL(CLI::GetUnmappedParam<arma::mat>("matrix"), "file1.csv");
  BOOST_REQUIRE_EQUAL(CLI::GetUnmappedParam<arma::mat>("matrix2"), "file2.csv");
  BOOST_REQUIRE_CLOSE(CLI::GetUnmappedParam<double>("double"), 1.334, 1e-10);
  BOOST_REQUIRE_SMALL(CLI::GetUnmappedParam<double>("double2"), 1e-10);
  BOOST_REQUIRE_EQUAL(CLI::GetUnmappedParam<GaussianKernel>("kernel"),
      "kernel.txt");
  BOOST_REQUIRE_EQUAL(CLI::GetUnmappedParam<GaussianKernel>("kernel2"),
      "kernel2.txt");

  // Can we assign an unmapped parameter?
  CLI::GetUnmappedParam<arma::mat>("matrix2") =
      CLI::GetUnmappedParam<arma::mat>("matrix");
  CLI::GetUnmappedParam<GaussianKernel>("kernel2") =
      CLI::GetUnmappedParam<GaussianKernel>("kernel");

  BOOST_REQUIRE_EQUAL(CLI::GetUnmappedParam<arma::mat>("matrix2"), "file1.csv");
  BOOST_REQUIRE_EQUAL(CLI::GetUnmappedParam<GaussianKernel>("kernel2"),
      "kernel.txt");

  remove("kernel.txt");
}

/**
 * Test that we can serialize a model and then deserialize it through the CLI
 * interface.
 */
BOOST_AUTO_TEST_CASE(SerializationTest)
{
  AddRequiredCLIOptions();

  CLI::Add<GaussianKernel>(GaussianKernel(), "kernel", "Test kernel", 'k',
      false, false);

  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "--kernel_file";
  argv[2] = "kernel.txt";

  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // Create the kernel we'll save.
  GaussianKernel gk(0.5);

  CLI::GetParam<GaussianKernel>("kernel") = move(gk);

  // Save it.
  CLI::Destroy();

  // Now create a new CLI object and load it.
  AddRequiredCLIOptions();

  CLI::Add<GaussianKernel>(GaussianKernel(), "kernel", "Test kernel", 'k',
      false, true);

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // Load the kernel from file.
  GaussianKernel gk2 = move(CLI::GetParam<GaussianKernel>("kernel"));

  BOOST_REQUIRE_CLOSE(gk2.Bandwidth(), 0.5, 1e-5);

  // Now remove the file we made.
  remove("kernel.txt");
}

/**
 * Test that an exception is thrown when a required model is not specified.
 */
BOOST_AUTO_TEST_CASE(RequiredModelTest)
{
  AddRequiredCLIOptions();

  CLI::Add<GaussianKernel>(GaussianKernel(), "kernel", "Test kernel", 'k', true,
      true);

  // Don't specify any input parameters.
  const char* argv[1];
  argv[0] = "./test";

  int argc = 1;

  Log::Fatal.ignoreInput = true;
  BOOST_REQUIRE_THROW(CLI::ParseCommandLine(argc, const_cast<char**>(argv)),
      runtime_error);
  Log::Fatal.ignoreInput = false;
}

/**
 * Test that we can load both a dataset and its associated info.
 */
BOOST_AUTO_TEST_CASE(MatrixAndDatasetInfoTest)
{
  AddRequiredCLIOptions();

  // Write test file to load.
  fstream f;
  f.open("test.arff", fstream::out);
  f << "@relation test" << endl;
  f << endl;
  f << "@attribute one STRING" << endl;
  f << "@attribute two REAL" << endl;
  f << endl;
  f << "@attribute three STRING" << endl;
  f << endl;
  f << "\% a comment line " << endl;
  f << endl;
  f << "@data" << endl;
  f << "hello, 1, moo" << endl;
  f << "cheese, 2.34, goodbye" << endl;
  f << "seven, 1.03e+5, moo" << endl;
  f << "hello, -1.3, goodbye" << endl;
  f.close();

  // Add options.
  typedef tuple<DatasetInfo, arma::mat> TupleType;
  CLI::Add<TupleType>(TupleType(), "dataset", "Test dataset", 'd', false,
      true);

  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "--dataset_file";
  argv[2] = "test.arff";

  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // Get the dataset and info.
  DatasetInfo info = move(get<0>(CLI::GetParam<TupleType>("dataset")));
  arma::mat dataset = move(get<1>(CLI::GetParam<TupleType>("dataset")));

  BOOST_REQUIRE_EQUAL(info.Dimensionality(), 3);

  BOOST_REQUIRE(info.Type(0) == Datatype::categorical);
  BOOST_REQUIRE_EQUAL(info.NumMappings(0), 3);
  BOOST_REQUIRE(info.Type(1) == Datatype::numeric);
  BOOST_REQUIRE(info.Type(2) == Datatype::categorical);
  BOOST_REQUIRE_EQUAL(info.NumMappings(2), 2);

  BOOST_REQUIRE_EQUAL(dataset.n_rows, 3);
  BOOST_REQUIRE_EQUAL(dataset.n_cols, 4);

  // The first dimension must all be different (except the ones that are the
  // same).
  BOOST_REQUIRE_EQUAL(dataset(0, 0), dataset(0, 3));
  BOOST_REQUIRE_NE(dataset(0, 0), dataset(0, 1));
  BOOST_REQUIRE_NE(dataset(0, 1), dataset(0, 2));
  BOOST_REQUIRE_NE(dataset(0, 2), dataset(0, 0));

  BOOST_REQUIRE_CLOSE(dataset(1, 0), 1.0, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(1, 1), 2.34, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(1, 2), 1.03e5, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(1, 3), -1.3, 1e-5);

  BOOST_REQUIRE_EQUAL(dataset(2, 0), dataset(2, 2));
  BOOST_REQUIRE_EQUAL(dataset(2, 1), dataset(2, 3));
  BOOST_REQUIRE_NE(dataset(2, 0), dataset(2, 1));

  remove("test.arff");
}

/**
 * Test that we can access a parameter before we load it.
 */
BOOST_AUTO_TEST_CASE(RawIntegralParameter)
{
  AddRequiredCLIOptions();

  CLI::Add<double>(0.0, "double", "Test double", 'd', false, true);

  const char* argv[1];
  argv[0] = "./test";
  int argc = 1;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // Set the double.
  CLI::GetRawParam<double>("double") = 3.0;

  // Now when we get it, it should be what we just set it to.
  BOOST_REQUIRE_CLOSE(CLI::GetParam<double>("double"), 3.0, 1e-5);
}

/**
 * Test that we can load a dataset with a pre-set mapping through
 * CLI::GetRawParam().
 */
BOOST_AUTO_TEST_CASE(RawDatasetInfoLoadParameter)
{
  AddRequiredCLIOptions();

  // Create the ARFF that we will read.
  fstream f;
  f.open("test.arff", fstream::out);
  f << "@relation test" << endl;
  f << endl;
  f << "@attribute one STRING" << endl;
  f << "@attribute two REAL" << endl;
  f << endl;
  f << "@attribute three STRING" << endl;
  f << endl;
  f << "\% a comment line " << endl;
  f << endl;
  f << "@data" << endl;
  f << "hello, 1, moo" << endl;
  f << "cheese, 2.34, goodbye" << endl;
  f << "seven, 1.03e+5, moo" << endl;
  f << "hello, -1.3, goodbye" << endl;
  f.close();

  CLI::Add<tuple<DatasetInfo, arma::mat>>(tuple<DatasetInfo, arma::mat>(),
      "tuple", "Test tuple", 't', false, true);

  const char* argv[3];
  argv[0] = "./test";
  argv[1] = "--tuple_file";
  argv[2] = "test.arff";
  int argc = 3;

  CLI::ParseCommandLine(argc, const_cast<char**>(argv));

  // Create a pre-filled DatasetInfo object.
  DatasetInfo info(3);
  info.Type(0) = Datatype::categorical;
  info.Type(2) = Datatype::categorical;
  info.MapString("seven", 0); // This will have mapped value 0.
  info.MapString("cheese", 0); // This will have mapped value 1.
  info.MapString("hello", 0); // This will have mapped value 2.
  info.MapString("goodbye", 2); // This will have mapped value 0.
  info.MapString("moo", 2); // This will have mapped value 1.

  // Now set the dataset info.
  std::get<0>(CLI::GetRawParam<tuple<DatasetInfo, arma::mat>>("tuple")) = info;

  // Now load the dataset.
  arma::mat dataset =
      std::get<1>(CLI::GetParam<tuple<DatasetInfo, arma::mat>>("tuple"));

  // Check the values.
  BOOST_REQUIRE_CLOSE(dataset(0, 0), 2.0, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(1, 0), 1.0, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(2, 0), 1.0, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(0, 1), 1.0, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(1, 1), 2.34, 1e-5);
  BOOST_REQUIRE_SMALL(dataset(2, 1), 1e-5);
  BOOST_REQUIRE_SMALL(dataset(0, 2), 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(1, 2), 1.03e+5, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(2, 2), 1.0, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(0, 3), 2.0, 1e-5);
  BOOST_REQUIRE_CLOSE(dataset(1, 3), -1.3, 1e-5);
  BOOST_REQUIRE_SMALL(dataset(2, 3), 1e-5);

  remove("test.arff");
}

BOOST_AUTO_TEST_SUITE_END();
