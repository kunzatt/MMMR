package com.ssafy.mmmr.todos.entity;

import java.time.LocalDateTime;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import com.ssafy.mmmr.profiles.entity.ProfileEntity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Table(name = "todos")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
public class TodoEntity {

	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private Long id;

	@ManyToOne
	@JoinColumn(name = "profile_id", nullable = false)
	private ProfileEntity profile;

	@Column(name = "content", nullable = false, length = 256)
	private String content;

	@Column(name = "is_done", nullable = false)
	private Boolean isDone;

	@CreatedDate
	@Column(name = "created_at", nullable = false)
	private LocalDateTime createdAt;

	@LastModifiedDate
	@Column(name = "updated_at", nullable = false)
	private LocalDateTime updatedAt;

	@Column(name = "deleted", nullable = false)
	private Boolean deleted;

	@Builder
	public TodoEntity(ProfileEntity profile, String content, Boolean isDone, Boolean deleted) {
		this.profile = profile;
		this.content = content;
		this.isDone = isDone;
		this.deleted = deleted;
	}

	public void delete() {
		this.deleted = true;
	}

	public void updateContent(String content) {
		this.content = content;
	}

	public void toggle() {
		this.isDone = !this.isDone;
	}
}
