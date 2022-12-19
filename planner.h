#ifndef PLANNER_H
#define PLANNER_H

#ifndef STD_HEADERS
#include "std_headers.h"
#endif

#ifndef DICTIONARY_H
#include "dictionary.h"
#endif

#ifndef TRIE_H
#include "Trie.h"
#endif

using namespace std;

class Planner {
 private:
  int numStations;
  int numJCodeSrcDestn;
  int lastReviewId;
  fstream logFile;

  StationAdjacencyList adjacency[DICT_SIZE];
  listOfObjects<TrainInfoPerStation *> *stationInfo[DICT_SIZE];
  Dictionary<int> stnNameToIndex;
  listOfObjects<JourneyCodeReview> *jCRMatrix[DICT_SIZE][DICT_SIZE];
  Trie *stnNamesTrie;

  inline int getUserLevel(int userCode);
  int getInt(bool readFrmFile, fstream *file);
  string getStringWithSpaces(bool readFrmFile, fstream *file);
  string getStringWithoutSpaces(bool readFrmFile, fstream *file);
  bool doAdminJob();
  bool doUserJob();
  listOfObjects<string> *findAllWords(string text);
  void clearJCRMatrixEntry(int row, int col);
  bool addStationName(string stnName);
  bool delStationName(string stnName);
  bool addJourneyCode(string srcStnName, string destStnName, int journeyCode);
  bool delJourneyCode(string srcStnName, string destStnName, int journeyCode);
  bool checkValidJourneyCode(string srcStnName, string destStnName, int journeyCode);
  int addReview(int journeyCode, string srcStnName, string destStnName, string reviewString, int rating);
  bool delReview(int reviewId);
  listOfObjects<JourneyCodeReview> *findJCodeReviews(string srcStnName, string dstStnName);
  inline void printWithHighlight(string text, int startHLight, int lenHLight);
  string chooseFromCompletions(listOfObjects<string> *completions);
  void trimByMatchingSubWords(listOfObjects<string> * &completions, listOfObjects<string> *listOfSubWords);
  int *computeHMatrixForKMP(string keywordString);
  int KMPMatch(string text, int *hMatrix, string pattern);
  void printDirectJourneys(string srcStnName, string destStnName);
  
 public:
  Planner(string logFileName) {
    numStations = 0;
    numJCodeSrcDestn = 0;
    lastReviewId = -1;
    logFile.open(logFileName, ios::out);
    if (!logFile.is_open()) {
      cout << "Critical error: Failed to open log file \"" << logFileName << "\"" << endl;
      exit(-1);
    }

    for (int i = 0; i < DICT_SIZE; i++) {
      stationInfo[i] = nullptr;
      for (int j = 0; j < DICT_SIZE; j++) {
	jCRMatrix[i][j] = nullptr;
      }
    }
    stnNamesTrie = new Trie(&stnNameToIndex);
  }

  ~Planner() {
    if (logFile.is_open()) {
      logFile.close();
    }
  }
  void displayWelcomeMessage();
  bool displayMenuAndAct();
  void Quicksort(listOfObjects<TrainInfoPerStation *> *stnInfoList);
  void QuicksortSimple(listOfObjects<TrainInfoPerStation *> *stnInfoList, int start, int end);
  void printStationInfo(listOfObjects<TrainInfoPerStation *> *stnInfoList);
};


#endif
