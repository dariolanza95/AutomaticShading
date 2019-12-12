
#ifndef DATALOADER_H
#define DATALOADER_H

/*

template <typename T>
class Array {
private:
    T *ptr;
    int size;
public:
    Array(T arr[], int s);
    void print();
};

template <typename T>
Array<T>::Array(T arr[], int s) {
    ptr = new T[s];
    size = s;
    for(int i = 0; i < size; i++)
        ptr[i] = arr[i];
}

template <typename T>
void Array<T>::print() {
    for (int i = 0; i < size; i++)
        cout<<" "<<*(ptr + i);
    cout<<endl;
}*/

template <typename T>
class DataLoader
{

public:
    DataLoader<T>();
    virtual T Load() = 0;

};

#endif // DATALOADER_H
