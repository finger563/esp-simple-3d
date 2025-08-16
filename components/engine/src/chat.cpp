#include "chat.hpp"

Chat_c::Chat_c()
    : place(conversation.rbegin()) {
  numChatsDisplayed = 4;
}

Chat_c::Chat_c(const int chats)
    : place(conversation.rbegin()) {
  numChatsDisplayed = chats;
}

void Chat_c::addMessage(std::string msg) {
  conversation.push_back(msg);
  place = conversation.rbegin();
}

void Chat_c::Up() { ++place; }

void Chat_c::Down() { --place; }

std::string Chat_c::getMsg() { return *conversation.rbegin(); }

Chat_c &Chat_c::operator=(const Chat_c &rhs) {
  if (this != &rhs) // protect against invalid self-assignment
  {
    conversation = rhs.Conversation();
    place = rhs.Place();
    numChatsDisplayed = rhs.NumChatsDisplayed();
  }
  return *this;
}
