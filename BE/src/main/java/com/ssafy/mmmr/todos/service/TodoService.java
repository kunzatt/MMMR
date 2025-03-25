package com.ssafy.mmmr.todos.service;

import org.springframework.stereotype.Service;

import com.ssafy.mmmr.todos.repository.TodoRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class TodoService {

	private final TodoRepository todoRepository;

}