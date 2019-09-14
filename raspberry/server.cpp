#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <chrono>
#include <vector>

#include "decoder.cpp"

using namespace boost::asio;
using boost::asio::ip::tcp;
using boost::system::error_code;
using std::chrono::milliseconds;

class session {
	tcp::socket s;
	enum { max_len = 1024 };
	char data[max_len];
	steady_timer tim;
	unsigned int counter;
	vector<string> str_vector;
	
	void handle_read(const error_code& ec, size_t sz) {
		if (!ec) {
			string aux_str = decode_message(data);
			if(!aux_str.empty())
				async_write(s, buffer(aux_str), boost::bind(&session::handle_write, this, _1, _2));
			else
				s.async_read_some(buffer(data, max_len), boost::bind(&session::handle_read, this, _1, _2 ));
		} else 
			delete this; 
	}
	
	void handle_write(const error_code& ec, size_t sz) {
		if (!ec) 
			s.async_read_some(buffer(data, max_len), boost::bind(&session::handle_read, this, _1, _2 ));
		else 
			delete this; 
	}
	
	void handle_write_timer(const error_code& ec, size_t sz) {
		if (!ec) {
			counter--;
			if (counter != 0)
				async_write(s, buffer(str_vector[counter]), boost::bind(&session::handle_write_timer, this, _1, _2));	
			else
				async_write(s, buffer(str_vector[counter]), boost::bind(&session::start_stream_timer, this));
		} else 
			delete this; 
	}
	
	void handle_stream_timer(const error_code & ec) {
		str_vector = real_time_stream();
		counter = str_vector.size();
		if (counter == 0) //no message in vector
			start_stream_timer();
		else {
			counter--;
			if (counter != 0)
				async_write(s, buffer(str_vector[counter]), boost::bind(&session::handle_write_timer, this, _1, _2));	
			else
				async_write(s, buffer(str_vector[counter]), boost::bind(&session::start_stream_timer, this));	
		} 
	}
	
	public:
		session(io_service& io) : s(io), tim(io) { }
		tcp::socket& socket() {
			return s;
		}
		
		void start_read_client() {
			async_write(s, buffer("Welcome!\n"), boost::bind(&session::handle_write, this, _1, _2));
		}
		
		void start_stream_timer() {
			tim.expires_from_now(milliseconds(500));
			tim.async_wait(boost::bind(&session::handle_stream_timer, this, _1));
		}

		void start() { 
			start_read_client();
			start_stream_timer();
		}
};

class server {
	io_service& io;
	tcp::acceptor acceptor;

	void start_accept() {
		session* new_sess = new session(io);
		acceptor.async_accept(new_sess->socket(), boost::bind(&server::handle_accept, this, new_sess,_1));
	}
	
	void handle_accept(session* sess, const error_code& ec) {
		if (!ec) 
			sess->start(); 
		else 
			delete sess;
		start_accept();
	}

	public:
		server(io_service& io, short port) : io(io), acceptor(io, tcp::endpoint(tcp::v4(), port)) {
			start_accept();
		}
};