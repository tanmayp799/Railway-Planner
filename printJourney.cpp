#ifndef PRINT_JOURNEY_CPP
#define PRINT_JOURNEY_CPP

#ifndef PLANNER_H
#include "planner.h"
#endif

#ifndef STD_HEADERS_H
#include "std_headers.h"
#endif

using namespace std;

#define CONST_GROWTH 1000
#define INIT_SIZE 1000


// Implemented a Circular Queue Data structure to implement BFS Algorithm efficiently
template <typename T>
class DynamicQueue
{
private:
  T *A;              // the array used for implementing the dynamic dequeue
  unsigned int N;    // the current size of array A
  unsigned int head; // index where next element of queue will be deleted from
  unsigned int tail; // index where next element will be inserted
  unsigned int nextSize() { return N + CONST_GROWTH; }

public:
  DynamicQueue() // default constructor
  {
    A = new T[INIT_SIZE];
    if (A != nullptr)
      N = INIT_SIZE;
    else
      N = 0;
    head = tail = 0;
  }
  ~DynamicQueue() { delete[] A; }; // default destructor

  bool isEmpty();      // is the queue empty?
  bool isFull();       // is the queue full?
  void grow();         // grow the queue to its next size
  unsigned int size(); // return the current number of elements in the queue
  void QInsert(T x);   // insert given element at tail of the queue; grow array size as necessary
  bool QDelete(T *x);  // delete element at head of the queue and place it in *T; returns false if queue is empty, true otherwise
  T returnHead() { return A[head]; }
  bool isPresent(T x);
};

template <typename T>
bool DynamicQueue<T>::isPresent(T x)
{
  for (int j = head; ((j) % N) != tail; j = (j + 1) % N)
  {
    if (x == A[j])
      return true;
  }
  return false;
}

template <typename T>
bool DynamicQueue<T>::isEmpty()
{
  return size() == 0; // If size = 0, empty
}

template <typename T>
bool DynamicQueue<T>::isFull()
{
  return size() == N; // If size = N, full
}

template <typename T>
void DynamicQueue<T>::grow()
{
  if (isFull())
  {
    unsigned int old = N;
    N = nextSize();
    T *p = new T[N]; // new array of larger size
    int i = 0, j = 0;
    for (i = 0, j = head; ((j + 1) % old) != tail; i++, j = (j + 1) % old)
    {
      p[i] = A[j]; // Copying element by element into new array
    }
    head = 0;     // Setting head at 0
    tail = i + 1; // Setting tail at N + 1
    delete[] A;   // Freeing up older array
    A = p;
  }
  else
    return;
}

template <typename T>
unsigned int DynamicQueue<T>::size()
{
  if (tail == -1)
    return N; // Return N if tail is dummy value
  else
    return (tail - head + N) % N;
}

template <typename T>
void DynamicQueue<T>::QInsert(T x)
{
  if (isFull())
    grow();

  A[tail] = x;
  tail = (tail + 1) % N;

  if (tail == head) // If N, set tail at -1 to resolve ambiguity
    tail = -1;
}

template <typename T>
bool DynamicQueue<T>::QDelete(T *x)
{
  if (isEmpty())
    return false;
  *x = A[head];

  if (tail == -1) // Queue no longer has N elements, hence "normal" use of tail
    tail = head;

  head = (head + 1) % N;
  return true;
}

//=================================================//

void Planner::printDirectJourneys(string srcStnName, string destStnName)
{

  // insert your code here
  listOfObjects<TrainInfoPerStation *> *validJourneys = nullptr; //The list of journeys we need to print

  DynamicQueue<int> validJC; //The list of valid JourneyCodes, i.e., the JCs coming to destination station.
  DynamicQueue<int> stnIndex; //The queue storing the station indices to be checked next in BFS.
  DynamicQueue<int> prevJC; //Stores the Train no.(JC) which took us to the current stnIndex. This helps keep in check that we only follow the paths having a direct route.

  Entry<int> *srcStn = stnNameToIndex.get(srcStnName); //Source station
  Entry<int> *destStn = stnNameToIndex.get(destStnName); //Destination station
  if (srcStn == nullptr)
  {
    cout << "INVALID SOURCE STATION NAME" << endl;
    if (destStn == nullptr)
    {
      cout << "INVALID DESTINATION STATION NAME" << endl;
    }
    return;
  }
  else
  {
    if (destStn == nullptr)
    {
      cout << "INVALID DESTINATION STATION NAME" << endl;
      return;
    }
  }

  listOfObjects<StationConnectionInfo *> *destAdjStn = adjacency[destStn->value].fromStations; //List of adjacent stations of destination station.

  int *currStnIndex = new int;//The index of current station being checked in BFS.
  int *currJC = new int; //The JC which took us to this current station.

  while (destAdjStn != nullptr) //Filling up the validJC list with valid JCs.
  {
    listOfObjects<TrainInfoPerStation *> *ts = destAdjStn->object->trains;
    while (ts != nullptr)
    {
      validJC.QInsert(ts->object->journeyCode);
      ts = ts->next;
    }
    destAdjStn = destAdjStn->next;
  }

  //Initializing BFS
  stnIndex.QInsert(srcStn->value); 
  prevJC.QInsert(0);


  /* **********      BFS           **************
    This modified version of BFS works as follows:-
      We keep adding the relevant stations to stnIndex queue. What are relevant stations? Well, the stations following a direct route till the Destination
      In the loop, we fetch and delete the first station in queue, check its adjacent stations for possible direct route.
      If a direct route is possible, we add the adjacent station to our stnIndex queue, and the journeyCode corresponding to this direct route in prevJC queue.
      The prevJC queue helps us make sure that direct route condition is satisfied.
      As we keep adding and removing stations, at a point the queue will only comprise of destination station.
      Here we start building our list of trains to print, and do this till stnIndex is empty.  
  */
  while (!stnIndex.isEmpty())
  {
    // int *currStnIndex = new int;
    stnIndex.QDelete(currStnIndex); //Fetch head of queue.
    prevJC.QDelete(currJC);//The JC from which we arrived to current station. Building list of trains.

    if(*currStnIndex==destStn->value) //Only destination station remains in stnIndex
    {
      listOfObjects<TrainInfoPerStation *> *ts = stationInfo[srcStn->value];//All trains at source.
      while(ts!=nullptr)
      {
        if(ts->object->journeyCode==*currJC) //To satisfy direct route condition
        {
          if(validJourneys==nullptr)//Corner case
          {
            
            listOfObjects<TrainInfoPerStation *> *t = new listOfObjects<TrainInfoPerStation *>(ts->object);
            validJourneys = t;
          }
          else
          {
            listOfObjects<TrainInfoPerStation *> *tmp = validJourneys;
            while(tmp->next!=nullptr) tmp = tmp->next;
           
            listOfObjects<TrainInfoPerStation *> *t = new listOfObjects<TrainInfoPerStation *>(ts->object);
            t->prev = tmp;
            tmp->next= t;
          }
        }
        ts = ts->next;
      }
      continue;
    }


    listOfObjects<StationConnectionInfo *> *currAdjStn = adjacency[*currStnIndex].toStations; //list of adjacent stations and trains coming towards them

    while (currAdjStn != nullptr) //Iterating through all the adjacent stations, implementing BFS basically.
    {
      listOfObjects<TrainInfoPerStation *> *trains = currAdjStn->object->trains; //List of trains at the current adjacent station being examined.

      while(trains!=nullptr) //Iterating through all trains at this station to find a direct path.
      {
        if(validJC.isPresent(trains->object->journeyCode)) //Conditon to capture train with a possible direct path
        {
          if(*currStnIndex==srcStn->value) //Corner case. Since we don't need to check the previousJC for source, 
                                            //we simply insert the relevant adjacent stations along with the JCs we got these stations from.

          {
            stnIndex.QInsert(currAdjStn->object->adjacentStnIndex);
            prevJC.QInsert(trains->object->journeyCode);
          }
          else
          {
            if(trains->object->journeyCode==*currJC) //Checking if the train has a direct oath, or if we have a possible but not DIRECT path. 
            {
              stnIndex.QInsert(currAdjStn->object->adjacentStnIndex);
              prevJC.QInsert(trains->object->journeyCode);
            }
          }

        }
        trains = trains->next;
      }
      currAdjStn = currAdjStn->next;
    }

  }

  if (validJourneys == nullptr) 
    cout << "No Direct Journeys Available" << endl;
  else
  {
    printStationInfo(validJourneys);
  }


  // Get the list of journeys as a listOfObjects<TrainInfoPerStation *>
  // for the source station and then we can print it out by invoking
  // printStationInfo that we had used in Lab 7.
  // printStationInfo is a private member function of the Planner class
  // It is declared in planner.h and implemented in planner.cpp

  return;
}

#endif
