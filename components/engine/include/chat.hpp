#pragma once

#include <iterator>
#include <string>
#include <vector>

class Chat_c {

private:
  std::vector<std::string> conversation;
  std::vector<std::string>::reverse_iterator place;

  int numChatsDisplayed;

public:
  Chat_c();
  Chat_c(const int chats);

  void NumChatsDisplayed(const int chats) { numChatsDisplayed = chats; }
  int NumChatsDisplayed() const { return numChatsDisplayed; }

  void Conversation(const std::vector<std::string> c) { conversation = c; }
  std::vector<std::string> Conversation() const { return conversation; }

  void Place(const std::vector<std::string>::reverse_iterator p) { place = p; }
  std::vector<std::string>::reverse_iterator Place() const { return place; }

  void addMessage(std::string msg);

  std::string getMsg();

  void Up();

  void Down();

  Chat_c &operator=(const Chat_c &rhs);
};
