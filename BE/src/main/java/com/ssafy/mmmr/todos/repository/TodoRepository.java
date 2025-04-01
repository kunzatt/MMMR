package com.ssafy.mmmr.todos.repository;

import java.util.List;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import com.ssafy.mmmr.todos.entity.TodoEntity;

@Repository
public interface TodoRepository extends JpaRepository<TodoEntity, Long> {

	List<TodoEntity> findByProfileIdAndDeletedFalse(Long profileId);

	List<TodoEntity> findByProfileIdAndIsDoneAndDeletedFalse(Long profileId, Boolean deleted);

}
