#include <time.h>

#include "CLATCHVocabulary.h"
using namespace std;

bool load_as_text(ORB_SLAM2::CLATCHVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  bool res = voc->loadFromTextFile(infile);
  printf("Loading fom text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return res;
}

void load_as_xml(ORB_SLAM2::CLATCHVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->load(infile);
  printf("Loading fom xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void load_as_binary(ORB_SLAM2::CLATCHVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->loadFromBinaryFile(infile);
  printf("Loading fom binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_xml(ORB_SLAM2::CLATCHVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->save(outfile);
  printf("Saving as xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_text(ORB_SLAM2::CLATCHVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToTextFile(outfile);
  printf("Saving as text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_binary(ORB_SLAM2::CLATCHVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToBinaryFile(outfile);
  printf("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}


int main(int argc, char **argv) {
  cout << "BoW load/save benchmark" << endl;
  //ORB_SLAM2::CLATCHVocabulary* voc = new ORB_SLAM2::CLATCHVocabulary();
  //load_as_text(voc, "Vocabulary/CLATCHvoc.txt");
  //save_as_binary(voc, "Vocabulary/CLATCHvoc.bin");

  ORB_SLAM2::CLATCHVocabulary* voc = new ORB_SLAM2::CLATCHVocabulary();
  load_as_xml(voc, "Vocabulary/small_clatch_voc.yml");
  save_as_binary(voc, "Vocabulary/CLATCHVoc.bin");
  //load_as_text(voc, "Vocabulary/CLATCHVoc.txt");
  //save_as_binary(voc, "Vocabulary/CLATCHVoc_txt.bin");

  return 0;
}
