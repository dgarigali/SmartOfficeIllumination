server:
	g++ -std=c++11 -lpigpio -lpthread -lrt -lboost_system -pthread main.cpp server.cpp desk.cpp decoder.cpp I2C.cpp -o server
	./server
	
client:
	g++ -std=c++11 -lboost_system client.cpp -o client
	./client
	
clean:
	rm server
	rm client