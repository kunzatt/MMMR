package com.ssafy.mmmr.todos.service;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.ProfileException;
import com.ssafy.mmmr.global.error.exception.TodoException;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.profiles.repository.ProfileRepository;
import com.ssafy.mmmr.todos.dto.TodoCreateRequestDto;
import com.ssafy.mmmr.todos.dto.TodoUpdateRequestDto;
import com.ssafy.mmmr.todos.dto.TodoResponseDto;
import com.ssafy.mmmr.todos.entity.TodoEntity;
import com.ssafy.mmmr.todos.repository.TodoRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class TodoService {

	private final TodoRepository todoRepository;
	private final ProfileRepository profileRepository;

	public List<TodoResponseDto> getAllTodosByProfileId(Long profileId) {
		return todoRepository.findByProfileIdAndDeletedFalse(profileId)
			.stream()
			.map(TodoResponseDto::fromEntity)
			.collect(Collectors.toList());
	}

	public Map<String, List<TodoResponseDto>> getTodosByProfileIdAndStatus(Long profileId) {
		List<TodoResponseDto> completedTodos = todoRepository.findByProfileIdAndIsDoneAndDeletedFalse(profileId, true)
			.stream()
			.map(TodoResponseDto::fromEntity)
			.collect(Collectors.toList());

		List<TodoResponseDto> incompleteTodos = todoRepository.findByProfileIdAndIsDoneAndDeletedFalse(profileId, false)
			.stream()
			.map(TodoResponseDto::fromEntity)
			.collect(Collectors.toList());

		Map<String, List<TodoResponseDto>> result = new HashMap<>();
		result.put("completedTodos", completedTodos);
		result.put("incompleteTodos", incompleteTodos);

		return result;
	}

	public TodoResponseDto getTodo(Long todoId) {
		TodoEntity todo = todoRepository.findById(todoId)
			.orElseThrow(() -> new TodoException(ErrorCode.TODO_NOT_FOUND));

		if (todo.getDeleted()) {
			throw new TodoException(ErrorCode.TODO_DELETED);
		}

		return TodoResponseDto.fromEntity(todo);
	}

	@Transactional
	public TodoResponseDto addTodo(TodoCreateRequestDto requestDto) {
		ProfileEntity profile = profileRepository.findById(requestDto.getProfileId())
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		TodoEntity todo = TodoEntity.builder()
			.profile(profile)
			.content(requestDto.getContent())
			.isDone(false)
			.deleted(false)
			.build();

		TodoEntity savedTodo = todoRepository.save(todo);
		return TodoResponseDto.fromEntity(savedTodo);
	}

	@Transactional
	public TodoResponseDto updateTodo(Long todoId, TodoUpdateRequestDto requestDto) {
		TodoEntity todo = todoRepository.findById(todoId)
			.orElseThrow(() -> new TodoException(ErrorCode.TODO_NOT_FOUND));

		if (todo.getDeleted()) {
			throw new TodoException(ErrorCode.TODO_DELETED);
		}

		todo.updateContent(requestDto.getContent());
		return TodoResponseDto.fromEntity(todo);
	}

	@Transactional
	public TodoResponseDto toggleTodoStatus(Long todoId) {
		TodoEntity todo = todoRepository.findById(todoId)
			.orElseThrow(() -> new TodoException(ErrorCode.TODO_NOT_FOUND));

		if (todo.getDeleted()) {
			throw new TodoException(ErrorCode.TODO_DELETED);
		}

		todo.toggle();
		return TodoResponseDto.fromEntity(todo);
	}

	@Transactional
	public void deleteTodo(Long todoId) {
		TodoEntity todo = todoRepository.findById(todoId)
			.orElseThrow(() -> new TodoException(ErrorCode.TODO_NOT_FOUND));

		todo.delete();
	}
}