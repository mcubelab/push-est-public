#include <iostream>
#include <json/json.h>
#include <fstream>
#include <sp_util.h>

using namespace std;

int main(){
  string filename = "/home/mcube/pushdata/allcontact_real_probe2-test.json";
  ifstream fin(filename);
  std::string jsonstr;

  fin.seekg(0, std::ios::end);
  jsonstr.reserve(fin.tellg());
  fin.seekg(0, std::ios::beg);

  jsonstr.assign((std::istreambuf_iterator<char>(fin)),
                  std::istreambuf_iterator<char>());

  Json::Value root;
  Json::Reader reader;
  bool parsingSuccessful = reader.parse( jsonstr, root );
  if ( !parsingSuccessful )
  {
    // report to the user the failure and their locations in the document.
    std::cout  << "Failed to parse configuration\n"
               << reader.getFormatedErrorMessages();


    return 1;
  }
  //std::string encoding = root.get("encoding", "UTF-8" ).asString();
  bool b =  root["isreal"].asBool();
  double muc = root["muc"].asDouble();
  cout << b << endl;
  cout << muc << endl;
  Json::Value allcontact_json = root["all_contact"];
  int n = allcontact_json.size();
  int nn = allcontact_json[0u].size();
  Matrix< double, Dynamic, Dynamic> all_contact(n, nn);

  for ( int i = 0; i < n; i++ )  // Iterates over the sequence elements.
    for ( int j = 0; j < nn; j++)
      all_contact(i,j) = allcontact_json[i][j].asDouble();

  cout << all_contact << endl;
}
